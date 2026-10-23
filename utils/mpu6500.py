"""MPU-6500 driver."""

import time

# --- register map ---------------------------------------------------------
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D
ACCEL_XOUT_H = 0x3B
USER_CTRL = 0x6A
WHO_AM_I = 0x75

I2C_IF_DIS = 0x10

# SPI: bit 7 of the address byte is the read flag.
SPI_READ = 0x80

SPI_DEAD_BYTES = (0x00, 0xFF)

# FS_SEL / AFS_SEL -> (register bits, LSB per unit)
GYRO_RANGES = {
    250: (0x00, 131.0),
    500: (0x08, 65.5),
    1000: (0x10, 32.8),
    2000: (0x18, 16.4),
}
ACCEL_RANGES = {
    2: (0x00, 16384.0),
    4: (0x08, 8192.0),
    8: (0x10, 4096.0),
    16: (0x18, 2048.0),
}

# WHO_AM_I values seen on genuine parts and the common clones
KNOWN_IDS = {
    0x68: "MPU-6050",
    0x70: "MPU-6500",
    0x71: "MPU-9250",
    0x72: "MPU-6555",
    0x73: "MPU-9255",
    0x98: "clone",
}

MPU6500_FAMILY = {0x70, 0x71, 0x72, 0x73}

TEMP_CAL = {
    0x68: (340.0, 36.53),
    0x70: (333.87, 21.0),
    0x71: (333.87, 21.0),
    0x72: (333.87, 21.0),
    0x73: (333.87, 21.0),
}
TEMP_CAL_DEFAULT = (333.87, 21.0)


class _I2CLink:
    """Register access over I2C (smbus2)."""

    kind = "i2c"

    def __init__(self, bus=1, address=0x68):
        from smbus2 import SMBus

        self.bus_num = bus
        self.address = address
        self.bus = SMBus(bus)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read_block(self, reg, count):
        return self.bus.read_i2c_block_data(self.address, reg, count)

    def describe(self):
        return f"I2C bus {self.bus_num} @ {self.address:#04x}"

    def close(self):
        self.bus.close()


class _SPILink:
    """Register access over SPI (spidev)."""

    kind = "spi"

    def __init__(self, bus=0, device=0, hz=1_000_000, mode=0):
        import spidev

        self.bus_num = bus
        self.device = device
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = int(hz)
        self.spi.mode = mode

    def read_byte(self, reg):
        return self.spi.xfer2([(reg & 0x7F) | SPI_READ, 0x00])[1]

    def write_byte(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value & 0xFF])

    def read_block(self, reg, count):
        return self.spi.xfer2([(reg & 0x7F) | SPI_READ] + [0x00] * count)[1:]

    def describe(self):
        return (
            f"SPI /dev/spidev{self.bus_num}.{self.device} "
            f"@ {self.spi.max_speed_hz / 1e6:g} MHz mode {self.spi.mode}"
        )

    def close(self):
        self.spi.close()


def open_link(
    transport="spi",
    bus=None,
    address=0x68,
    spi_bus=0,
    spi_device=0,
    spi_hz=1_000_000,
    spi_mode=0,
):
    """Build the transport named by `transport` ("spi" or "i2c")."""
    if transport == "spi":
        return _SPILink(spi_bus, spi_device, spi_hz, spi_mode)
    if transport == "i2c":
        return _I2CLink(1 if bus is None else bus, address)
    raise ValueError(f"transport must be 'spi' or 'i2c', not {transport!r}")


class MPU6500:
    def __init__(
        self,
        bus=None,
        address=0x68,
        gyro_range=1000,
        accel_range=4,
        transport="spi",
        spi_bus=0,
        spi_device=0,
        spi_hz=1_000_000,
        spi_mode=0,
        link=None,
    ):
        self.address = address
        self.bus_num = bus
        self.link = (
            link
            if link is not None
            else open_link(
                transport, bus, address, spi_bus, spi_device, spi_hz, spi_mode
            )
        )
        self.transport = self.link.kind
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.chip_id = None
        self._temp_div, self._temp_off = TEMP_CAL_DEFAULT

        if gyro_range not in GYRO_RANGES:
            raise ValueError(f"gyro_range must be one of {sorted(GYRO_RANGES)}")
        if accel_range not in ACCEL_RANGES:
            raise ValueError(f"accel_range must be one of {sorted(ACCEL_RANGES)}")
        self.gyro_range = gyro_range
        self.accel_range = accel_range
        self._gyro_bits, self._gyro_scale = GYRO_RANGES[gyro_range]
        self._accel_bits, self._accel_scale = ACCEL_RANGES[accel_range]

    # --- low level --------------------------------------------------------
    def who_am_i(self):
        self.chip_id = self.link.read_byte(WHO_AM_I)
        self._temp_div, self._temp_off = TEMP_CAL.get(self.chip_id, TEMP_CAL_DEFAULT)
        return self.chip_id

    def chip_name(self):
        return KNOWN_IDS.get(self.chip_id, f"unknown ({self.chip_id:#04x})")

    def is_6500(self):
        """True for the MPU-6500/9250 family, which needs ACCEL_CONFIG2."""
        if self.chip_id is None:
            self.who_am_i()
        return self.chip_id in MPU6500_FAMILY

    def wake(self):
        """Reset, clear sleep, select the PLL clock, set ranges, DLPF and rate."""
        if self.chip_id is None:
            self.who_am_i()
        self.link.write_byte(PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        if self.transport == "spi":
            self.link.write_byte(USER_CTRL, I2C_IF_DIS)
        self.link.write_byte(PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.link.write_byte(PWR_MGMT_1, 0x01)  # gyro-X PLL
        self.link.write_byte(CONFIG, 0x03)  # DLPF ~44 Hz
        # Internal output rate 1 kHz, so a poll never returns a stale sample.
        self.link.write_byte(SMPLRT_DIV, 0x00)
        self.link.write_byte(GYRO_CONFIG, self._gyro_bits)
        self.link.write_byte(ACCEL_CONFIG, self._accel_bits)
        if self.is_6500():
            self.link.write_byte(ACCEL_CONFIG2, 0x03)
        time.sleep(0.1)

    # --- measurements -----------------------------------------------------
    def read_raw(self):
        """Burst-read all 14 sensor bytes so the sample is time-coherent."""
        d = self.link.read_block(ACCEL_XOUT_H, 14)

        def word(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v > 32767 else v

        return (word(0), word(2), word(4), word(6), word(8), word(10), word(12))

    def read(self):
        """Return accel in g, gyro in deg/s (bias removed), temp in C."""
        ax, ay, az, temp, gx, gy, gz = self.read_raw()
        bx, by, bz = self.gyro_bias
        a, g = self._accel_scale, self._gyro_scale
        return {
            "ax": ax / a,
            "ay": ay / a,
            "az": az / a,
            "gx": gx / g - bx,
            "gy": gy / g - by,
            "gz": gz / g - bz,
            "temp": temp / self._temp_div + self._temp_off,
        }

    def describe(self):
        return self.link.describe()

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass
