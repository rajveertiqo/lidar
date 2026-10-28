"""BlueZ GATT server for the Shappe BLE bridge — PROTOTYPE.

STATUS: written against the BlueZ D-Bus API (BlueZ 5.82 on this Pi) and checked
with `--check` only. It has NOT yet advertised or served a phone: that needs the
adapter un-blocked and a supervised first run (docs/04 §8, docs/09 Phase 5).

It is a SEPARATE PROCESS from the navigation service on purpose: it only reads
the service's loopback HTTP API and reports an expiring phone audio-readiness lease.
A Bluetooth fault does not block the guidance loop.

    venv/bin/python -m ble_bridge.gatt_server --check            # D-Bus + adapter, no radio
    venv/bin/python -m ble_bridge.gatt_server --mock sidewalk    # a fake device, no sensors
    venv/bin/python -m ble_bridge.gatt_server                    # the live navigation service
"""

import argparse
import math
import os
import platform
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

from . import protocol as P
from .core import Bridge, Out, load_vocab
from .sources import SCENARIOS, HttpSource, MockSource

BLUEZ = "org.bluez"
ADAPTER_IFACE = "org.bluez.Adapter1"
DEVICE_IFACE = "org.bluez.Device1"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADV_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"
AGENT_IFACE = "org.bluez.Agent1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
LE_ADV_IFACE = "org.bluez.LEAdvertisement1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"

APP_PATH = "/org/shappe/ble"
ADV_PATH = APP_PATH + "/advertisement0"
AGENT_PATH = APP_PATH + "/agent"

POLL_S = 0.1  # /guidance/walk: the loop ticks at 5 Hz; polling at 10 Hz adds <=100 ms
HEALTH_S = 1.0  # /health, /guidance/status and the Pi's thermals

# Characteristic flags per security level (BlueZ org.bluez.GattCharacteristic1 "Flags").
# "encrypt-*" needs an encrypted link (any bond, Just Works included) and answers an
# unencrypted request with ATT Insufficient Encryption (0x0F); "encrypt-authenticated-*"
# needs a MITM-protected bond (passkey / numeric comparison) and answers 0x05.
# "open" is for a desk test only — it lets ANY nearby phone read the alerts.
SECURITY = {
    "open": {
        "read": ["read"],
        "notify": ["read", "notify"],
        "notify_only": ["notify"],
        "write": ["write"],
    },
    "encrypt": {
        "read": ["encrypt-read"],
        "notify": ["encrypt-read", "encrypt-notify"],
        "notify_only": ["encrypt-notify"],
        "write": ["encrypt-write"],
    },
    "authenticated": {
        "read": ["encrypt-authenticated-read"],
        "notify": ["encrypt-authenticated-read", "encrypt-authenticated-notify"],
        "notify_only": ["encrypt-authenticated-notify"],
        "write": ["encrypt-authenticated-write"],
    },
}
MAX_NAME_BYTES = 8  # legacy advert: flags 3 + 128-bit UUID 18 + name (2 + n) <= 31


class InvalidArgs(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"


class NotSupported(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


class InvalidOffset(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.InvalidOffset"


class Rejected(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.Rejected"


class NotAuthorized(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotAuthorized"


def _bytes(value: bytes) -> dbus.Array:
    return dbus.Array([dbus.Byte(b) for b in value], signature="y")


class Application(dbus.service.Object):
    def __init__(self, bus):
        self.services: List["Service"] = []
        super().__init__(bus, APP_PATH)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        objects = {}
        for svc in self.services:
            objects[svc.get_path()] = svc.get_properties()
            for chrc in svc.characteristics:
                objects[chrc.get_path()] = chrc.get_properties()
        return objects


class Service(dbus.service.Object):
    def __init__(self, bus, index: int, uuid: str):
        self.path = f"{APP_PATH}/service{index}"
        self.uuid = uuid
        self.characteristics: List["Characteristic"] = []
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": True,
                "Characteristics": dbus.Array(
                    [c.get_path() for c in self.characteristics], signature="o"
                ),
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgs()
        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    def __init__(
        self,
        bus,
        service: Service,
        index: int,
        uuid: str,
        flags: List[str],
        read: Optional[Callable[[], bytes]] = None,
        write: Optional[Callable[[bytes, Dict[str, Any]], None]] = None,
        on_subscribe: Optional[Callable[[], None]] = None,
    ):
        self.path = f"{service.path}/char{index}"
        self.service, self.uuid, self.flags = service, uuid, flags
        self.read_cb, self.write_cb, self.on_subscribe = read, write, on_subscribe
        self.notifying = False
        super().__init__(bus, self.path)
        service.characteristics.append(self)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": dbus.Array(self.flags, signature="s"),
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgs()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        if self.read_cb is None:
            raise NotSupported()
        value = self.read_cb() or b""
        offset = int(options.get("offset", 0))  # GATT long read (Read Blob)
        if offset > len(value):
            raise InvalidOffset()
        return _bytes(value[offset:])

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, options):
        if self.write_cb is None:
            raise NotSupported()
        self.write_cb(bytes(bytearray(value)), dict(options))

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        if self.on_subscribe:
            self.on_subscribe()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        self.notifying = False

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def notify(self, value: bytes) -> None:
        if self.notifying:
            self.PropertiesChanged(GATT_CHRC_IFACE, {"Value": _bytes(value)}, [])


class Advertisement(dbus.service.Object):
    """Legacy advertising: flags (3) + 128-bit service UUID (18) + name (2+len) <= 31 bytes,
    so the name is at most 8 bytes; a longer one is shortened by BlueZ."""

    def __init__(self, bus, name: str):
        self.name = name
        super().__init__(bus, ADV_PATH)

    def get_properties(self):
        return {
            LE_ADV_IFACE: {
                "Type": "peripheral",
                "ServiceUUIDs": dbus.Array([P.SERVICE_UUID], signature="s"),
                "LocalName": dbus.String(self.name),
                "Discoverable": dbus.Boolean(True),
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADV_IFACE:
            raise InvalidArgs()
        return self.get_properties()[LE_ADV_IFACE]

    @dbus.service.method(LE_ADV_IFACE, in_signature="", out_signature="")
    def Release(self):
        print("[ble] advertisement released by BlueZ")


class Agent(dbus.service.Object):
    """Pairing agent. NoInputNoOutput = LE Secure Connections 'Just Works' (no MITM
    protection) — acceptable for a supervised prototype only; see docs/04 §6."""

    def __init__(self, bus, window_open: Callable[[], bool]):
        self.window_open = window_open
        super().__init__(bus, AGENT_PATH)

    def _gate(self, what: str, device) -> None:
        if not self.window_open():
            print(f"[ble] refused {what} from {device}: pairing window closed")
            raise Rejected("pairing window closed")
        print(f"[ble] allowed {what} from {device}")

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        pass

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        self._gate(f"service {uuid}", device)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        self._gate("pairing", device)

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        self._gate("pairing confirmation", device)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        raise Rejected("no PIN on this device")

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        raise Rejected("no keyboard on this device")

    @dbus.service.method(AGENT_IFACE, in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        pass

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        pass

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Cancel(self):
        print("[ble] pairing cancelled")


def find_adapter(bus, name: Optional[str]) -> Optional[str]:
    om = dbus.Interface(bus.get_object(BLUEZ, "/"), DBUS_OM_IFACE)
    for path, ifaces in om.GetManagedObjects().items():
        if GATT_MANAGER_IFACE in ifaces and LE_ADV_MANAGER_IFACE in ifaces:
            if name is None or path.endswith("/" + name):
                return str(path)
    return None


def _identity(mock: bool) -> Dict[str, Any]:
    def _cmd(*argv: str) -> str:
        try:
            return subprocess.run(
                argv, capture_output=True, text=True, timeout=2
            ).stdout.strip()
        except Exception:
            return ""

    try:
        with open("/proc/device-tree/model") as fh:
            hw = fh.read().strip("\x00\n ")
    except OSError:
        hw = platform.machine()
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sha = _cmd("git", "-C", repo, "rev-parse", "--short", "HEAD") or "unknown"
    dirty = "+dirty" if _cmd("git", "-C", repo, "status", "--porcelain") else ""
    return {
        "mock": mock,
        "manufacturer": "Shappe project (prototype)",
        "model": "Shappe Guide MOCK" if mock else "Shappe Guide (Raspberry Pi 5)",
        "hardware": hw,
        "firmware": platform.release(),
        "software": f"guide {sha}{dirty}; ble-bridge 0.1; protocol {P.PROTOCOL_VERSION}",
    }


class Server:
    def __init__(
        self,
        bus,
        adapter: str,
        bridge: Bridge,
        source: Any,
        security: str,
        pair_window_s: float,
        name: str,
    ):
        self.bus, self.adapter, self.bridge, self.source = bus, adapter, bridge, source
        self.name = name
        self.pair_until = time.monotonic() + pair_window_s
        self.seq = {"alert": 0, "state": 0, "events": 0}
        self.reassembler = P.Reassembler()
        self.connected: set = set()
        self.stop = threading.Event()
        flags = SECURITY[security]
        ident = bridge.identity

        self.app = Application(bus)
        dis = Service(bus, 0, P.DIS_UUID)
        for i, (uuid, value) in enumerate(
            (
                (P.DIS_MANUFACTURER_UUID, ident["manufacturer"]),
                (P.DIS_MODEL_UUID, ident["model"]),
                (P.DIS_HARDWARE_REV_UUID, ident["hardware"]),
                (P.DIS_FIRMWARE_REV_UUID, ident["firmware"]),
                (P.DIS_SOFTWARE_REV_UUID, ident["software"]),
            )
        ):
            Characteristic(
                bus, dis, i, uuid, ["read"], read=lambda v=value: v.encode()[:180]
            )
        nav = Service(bus, 1, P.SERVICE_UUID)
        self.chars = {
            "info": Characteristic(
                bus, nav, 0, P.PROTOCOL_INFO_UUID, flags["read"], read=self._read_info
            ),
            "alert": Characteristic(
                bus, nav, 1, P.ALERT_UUID, flags["notify"], read=self._read_alert
            ),
            "state": Characteristic(
                bus,
                nav,
                2,
                P.STATE_UUID,
                flags["notify"],
                read=self._read_state,
                on_subscribe=lambda: self.send(self.bridge.tick(force=True)),
            ),
            "command": Characteristic(
                bus, nav, 3, P.COMMAND_UUID, flags["write"], write=self._on_write
            ),
            "events": Characteristic(bus, nav, 4, P.EVENTS_UUID, flags["notify_only"]),
        }
        self.app.services += [dis, nav]
        self.adv = Advertisement(bus, name)
        self.agent = Agent(bus, self._window_open)
        if pair_window_s > 0:
            GLib.timeout_add(int(pair_window_s * 1000) + 100, self._window_closed)

    # --- who may use the bridge ---

    def _window_open(self) -> bool:
        return time.monotonic() < self.pair_until

    def _bonded(self, path) -> bool:
        try:
            props = dbus.Interface(self.bus.get_object(BLUEZ, path), DBUS_PROP_IFACE)
            return bool(props.Get(DEVICE_IFACE, "Bonded"))
        except dbus.exceptions.DBusException:
            return False

    def _may_use(self, path) -> bool:
        """Anyone inside the pairing window; after it, only a bonded phone. BlueZ's
        Pairable stops new BONDS only: pairing without bonding still encrypts the link,
        which is all the characteristics ask, at any time (scan 2026-10-23)."""
        return self._window_open() or self._bonded(path)

    def _turn_away(self, path) -> bool:
        """Disconnect `path` if it may not use the bridge. False: as a GLib callback, once."""
        if path in self.connected and not self._may_use(path):
            print(
                f"[ble] disconnecting {self._describe(path)}: not bonded, "
                f"and the pairing window is closed"
            )
            try:
                dbus.Interface(
                    self.bus.get_object(BLUEZ, path), DEVICE_IFACE
                ).Disconnect()
            except dbus.exceptions.DBusException as exc:
                print(f"[ble] could not disconnect {path}: {exc}")
        return False

    def _window_closed(self) -> bool:
        print("[ble] pairing window closed: bonded phones only from now on")
        for path in list(
            self.connected
        ):  # one that connected inside it but never bonded
            self._turn_away(path)
        return False

    # --- reads: one whole frame, served by GATT long reads if over the MTU ---

    def _read_info(self) -> bytes:
        return P.encode(
            P.T_PROTOCOL_INFO, P.json_bytes(self.bridge.protocol_info()), 0, 517
        )[0]

    def _read_alert(self) -> bytes:
        rec = self.bridge.alert_value()
        if rec is None:
            rec = P.pack_alert(
                self.bridge.epoch,
                0,
                None,
                P.PHRASE_NONE,
                P.LEVEL_NONE,
                P.AF_REPLAY,
                None,
            )
        return P.encode(P.T_ALERT, rec, 0)[0]

    def _read_state(self) -> bytes:
        return P.encode(P.T_STATE, self.bridge.state_value(), 0)[0]

    # --- commands ---

    def _on_write(self, value: bytes, options: Dict[str, Any]) -> None:
        device = options.get("device")
        if device is not None and not self._may_use(device):
            raise NotAuthorized("not bonded, and the pairing window is closed")
        mtu = options.get("mtu")
        if mtu:
            self.bridge.set_mtu(
                link=int(mtu)
            )  # what BlueZ negotiated caps the app's claim
        try:
            done = self.reassembler.feed(value, time.monotonic())
        except P.ProtocolError as exc:
            self.send(self.bridge.bad_frame(exc.code, str(exc)))
            return
        if done is None:
            return
        msg_type, _seq, payload = done
        if msg_type != P.T_COMMAND:
            self.send(
                self.bridge.bad_frame(
                    "BAD_FRAME", f"type 0x{msg_type:02x} is not a command"
                )
            )
            return
        # A command may wait on HTTP: never on the D-Bus thread.
        threading.Thread(
            target=lambda: self.post(self.bridge.on_command(payload)),
            name="ble-command",
            daemon=True,
        ).start()

    # --- sending (D-Bus main loop thread only) ---

    def send(self, outs: List[Out]) -> bool:
        for char, msg_type, payload in outs:
            if not self.chars[char].notifying:
                continue  # nobody subscribed: nothing is queued and no seq is used up
            try:
                self._send_one(char, msg_type, payload)
            except (
                Exception
            ) as exc:  # one bad message must not lose the rest (STATE, alerts)
                self.bridge.log(
                    "send",
                    f"{char} message 0x{msg_type:02x} not sent: "
                    f"{type(exc).__name__}: {exc}",
                )
        return False  # as a GLib idle callback: run once

    def _send_one(self, char: str, msg_type: int, payload: bytes) -> None:
        mtu = self.bridge.att_mtu
        try:
            frames = P.encode(msg_type, payload, self.seq[char], mtu)
        except P.ProtocolError as exc:
            if msg_type != P.T_RESPONSE:
                raise
            # A reply too big for this link (the MTU can fall after it was built): the app
            # still gets an answer to its command, as TOO_LARGE.
            self.bridge.log("send-reply", f"reply not sent ({exc}); answered TOO_LARGE")
            frames = P.encode(
                msg_type, self.bridge.too_large(payload, str(exc)), self.seq[char], mtu
            )
        self.seq[char] = (self.seq[char] + 1) & 0xFFFF
        for frame in frames:
            self.chars[char].notify(frame)

    def post(self, outs: List[Out]) -> None:
        if outs:
            GLib.idle_add(self.send, outs)

    # --- polling the navigation service (its own thread) ---

    def poll_loop(self) -> None:
        """Runs until stop. Nothing may end it: the process would live on, the phone silent."""
        next_health = 0.0
        while not self.stop.is_set():
            started = time.monotonic()
            outs: List[Out] = []
            try:
                try:
                    outs += self.bridge.on_walk(self.source.walk())
                except Exception as exc:
                    next_health = 0.0  # once it answers again, ask /health at once
                    outs += self.bridge.on_unreachable(f"{type(exc).__name__}: {exc}")
                else:
                    if started >= next_health:
                        next_health = started + HEALTH_S
                        self.report_delivery()
                        try:
                            outs += self.bridge.on_health(
                                self.source.health(),
                                self.source.system_health(),
                                self.source.system(),
                            )
                        except Exception as exc:
                            outs += self.bridge.on_unreachable(
                                f"{type(exc).__name__}: {exc}"
                            )
                outs += self.bridge.tick()
            except (
                Exception
            ) as exc:  # a bug or a value nobody expected: log it, poll again
                self.bridge.log("poll", f"poll loop: {type(exc).__name__}: {exc}")
            self.post(outs)
            self.stop.wait(max(0.0, POLL_S - (time.monotonic() - started)))

    # --- connections ---

    def report_delivery(self) -> None:
        """Report independently of sensor health; a missing report expires in the service."""
        if not hasattr(self.source, "delivery_status"):
            return  # mock source: no live service to update
        connected = len(self.connected) == 1
        ready = connected and self.chars["alert"].notifying and self.bridge.phone_audio_ready()
        try:
            self.source.delivery_status(connected, bool(ready))
        except Exception as exc:
            self.bridge.log("delivery", f"delivery status: {type(exc).__name__}: {exc}")

    def on_device_props(self, interface, changed, invalidated, path=None) -> None:
        if interface == DEVICE_IFACE and "Connected" in changed:
            self.set_connected(path, bool(changed["Connected"]))

    def on_interfaces_added(self, path, interfaces) -> None:
        # A phone BlueZ has not seen before (first contact, or a new private address)
        # appears as a new Device1 that is already connected: no PropertiesChanged follows.
        props = interfaces.get(DEVICE_IFACE)
        if props is not None and bool(props.get("Connected", False)):
            self.set_connected(path, True)

    def on_interfaces_removed(self, path, interfaces) -> None:
        if DEVICE_IFACE in interfaces:
            self.set_connected(path, False)

    def set_connected(self, path, connected: bool) -> None:
        if connected == (path in self.connected):
            return  # already known: one connect and one disconnect each
        who = self._describe(path)
        if connected:
            self.connected.add(path)
            self.seq = {k: 0 for k in self.seq}
            self.reassembler = P.Reassembler()
            self.bridge.on_connect()
            print(f"[ble] connected: {who} ({len(self.connected)} connected)")
            GLib.idle_add(self._turn_away, path)
            if len(self.connected) > 1:
                print("[ble] WARNING: more than one phone connected; v1 supports one")
        else:
            self.connected.discard(path)
            self.bridge.on_disconnect()
            print(f"[ble] disconnected: {who}")

    def _describe(self, path) -> str:
        # Pairing swaps a phone's private address for its identity address but BlueZ keeps
        # the object path until it restarts, so log the current Alias and Address.
        try:
            props = dbus.Interface(self.bus.get_object(BLUEZ, path), DBUS_PROP_IFACE)
            return f"{props.Get(DEVICE_IFACE, 'Alias')} {props.Get(DEVICE_IFACE, 'Address')}"
        except dbus.exceptions.DBusException:
            return str(path)  # already removed


def check(bus, adapter_name: Optional[str]) -> int:
    """Read-only: is BlueZ reachable, is there an LE-capable adapter, is it usable?"""
    adapter = find_adapter(bus, adapter_name)
    if adapter is None:
        print("FAIL: no adapter offers GattManager1 + LEAdvertisingManager1")
        return 2
    props = dbus.Interface(bus.get_object(BLUEZ, adapter), DBUS_PROP_IFACE)
    a = props.GetAll(ADAPTER_IFACE)
    adv = props.GetAll(LE_ADV_MANAGER_IFACE)
    print(f"adapter      {adapter} {a.get('Address')} alias={a.get('Alias')}")
    print(
        f"powered      {bool(a.get('Powered'))} (PowerState={a.get('PowerState', '?')})"
    )
    print(
        f"pairable     {bool(a.get('Pairable'))}  discoverable={bool(a.get('Discoverable'))}"
    )
    print(f"roles        {[str(r) for r in a.get('Roles', [])]}")
    print(
        f"advertising  {int(adv.get('ActiveInstances', 0))} active of "
        f"{int(adv.get('SupportedInstances', 0))}; includes={[str(i) for i in adv.get('SupportedIncludes', [])]}"
    )
    if not a.get("Powered"):
        print(
            "NOTE: adapter is off. If PowerState is off-blocked: sudo rfkill unblock bluetooth"
        )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Shappe BLE bridge (PROTOTYPE)")
    ap.add_argument(
        "--check", action="store_true", help="inspect BlueZ and the adapter, then exit"
    )
    ap.add_argument(
        "--mock", choices=sorted(SCENARIOS), help="serve a scripted fake device"
    )
    ap.add_argument(
        "--mock-audio",
        choices=("on", "off"),
        default="on",
        help="mock: the device's AUDIO_ENABLED (off = the app must make no sound)",
    )
    ap.add_argument(
        "--mock-device-voice",
        action="store_true",
        help="mock: pretend the Pi's own voice speaks every cue (app must not repeat it)",
    )
    ap.add_argument(
        "--url", default="http://127.0.0.1:8000", help="navigation service base URL"
    )
    ap.add_argument(
        "--adapter", default=None, help="e.g. hci0 (default: the first LE adapter)"
    )
    ap.add_argument(
        "--name",
        default=None,
        help="advertised name, <= 8 bytes (default Shappe / Shappe-M)",
    )
    ap.add_argument("--security", choices=sorted(SECURITY), default="encrypt")
    ap.add_argument(
        "--pair-window",
        type=float,
        default=180.0,
        help="seconds after start during which a new phone may pair (0: none; "
        "after it, only bonded phones may connect)",
    )
    ap.add_argument(
        "--state-file",
        default=os.path.expanduser("~/.local/state/shappe-ble/state.json"),
    )
    args = ap.parse_args()

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    if args.check:
        return check(bus, args.adapter)

    adapter = find_adapter(bus, args.adapter)
    if adapter is None:
        print("[ble] no LE adapter found (GattManager1 + LEAdvertisingManager1)")
        return 2
    adapter_props = dbus.Interface(bus.get_object(BLUEZ, adapter), DBUS_PROP_IFACE)
    try:
        adapter_props.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
    except dbus.exceptions.DBusException as exc:
        print(
            f"[ble] cannot power the adapter: {exc.get_dbus_message()}\n"
            f"      if it is rfkill-blocked: sudo rfkill unblock bluetooth"
        )
        return 3

    mock = args.mock is not None
    source = (
        MockSource(
            args.mock,
            audio_enabled=args.mock_audio == "on",
            device_voice=args.mock_device_voice,
        )
        if mock
        else HttpSource(args.url)
    )
    bridge = Bridge(
        load_vocab(), _identity(mock), service=source, state_path=args.state_file
    )
    name = args.name or ("Shappe-M" if mock else "Shappe")
    if len(name.encode()) > MAX_NAME_BYTES:
        print(
            f"[ble] WARNING: name '{name}' is over {MAX_NAME_BYTES} bytes; BlueZ will shorten it"
        )
    server = Server(bus, adapter, bridge, source, args.security, args.pair_window, name)

    adapter_props.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(args.pair_window > 0))
    if args.pair_window > 0:  # a PairableTimeout of 0 is BlueZ's "pairable for ever"
        adapter_props.Set(
            ADAPTER_IFACE,
            "PairableTimeout",
            dbus.UInt32(max(1, int(math.ceil(args.pair_window)))),
        )

    agents = dbus.Interface(bus.get_object(BLUEZ, "/org/bluez"), AGENT_MANAGER_IFACE)
    agents.RegisterAgent(AGENT_PATH, "NoInputNoOutput")
    agents.RequestDefaultAgent(AGENT_PATH)

    loop = GLib.MainLoop()
    failed = []

    def _registered(what):
        return lambda: print(f"[ble] {what} registered")

    def _failed(what):
        def cb(error):
            print(f"[ble] {what} registration failed: {error}")
            failed.append(what)
            loop.quit()

        return cb

    gatt = dbus.Interface(bus.get_object(BLUEZ, adapter), GATT_MANAGER_IFACE)
    advm = dbus.Interface(bus.get_object(BLUEZ, adapter), LE_ADV_MANAGER_IFACE)
    gatt.RegisterApplication(
        APP_PATH,
        {},
        reply_handler=_registered("GATT application"),
        error_handler=_failed("GATT application"),
    )
    advm.RegisterAdvertisement(
        ADV_PATH,
        {},
        reply_handler=_registered(f"advertisement '{name}'"),
        error_handler=_failed("advertisement"),
    )
    bus.add_signal_receiver(
        server.on_device_props,
        dbus_interface=DBUS_PROP_IFACE,
        signal_name="PropertiesChanged",
        arg0=DEVICE_IFACE,
        path_keyword="path",
    )
    bus.add_signal_receiver(
        server.on_interfaces_added,
        dbus_interface=DBUS_OM_IFACE,
        signal_name="InterfacesAdded",
        bus_name=BLUEZ,
    )
    bus.add_signal_receiver(
        server.on_interfaces_removed,
        dbus_interface=DBUS_OM_IFACE,
        signal_name="InterfacesRemoved",
        bus_name=BLUEZ,
    )

    def _bluez_owner(owner):
        # bluetoothd restarted or stopped: our registrations died with it. Exit non-zero
        # so systemd (Restart=always) starts a fresh bridge that registers again.
        if not owner:
            print(
                "[ble] bluetoothd left the system bus; exiting so the bridge is restarted"
            )
            failed.append("bluetoothd")
            loop.quit()

    bus.watch_name_owner(BLUEZ, _bluez_owner)

    poller = threading.Thread(target=server.poll_loop, name="ble-poll", daemon=True)
    poller.start()

    def _quit(*_):
        server.stop.set()
        server.send(bridge._event("shutting_down", {}))
        loop.quit()
        return False

    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, _quit)
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, _quit)
    print(
        f"[ble] {'MOCK ' + args.mock if mock else 'live ' + args.url}; security={args.security}; "
        + (
            f"pairing open for {args.pair_window:.0f} s"
            if args.pair_window > 0
            else "pairing closed: bonded phones only"
        )
        + f"; device_id={bridge.device_id} epoch={bridge.epoch}"
    )
    loop.run()

    for call in (
        lambda: advm.UnregisterAdvertisement(ADV_PATH),
        lambda: gatt.UnregisterApplication(APP_PATH),
        lambda: agents.UnregisterAgent(AGENT_PATH),
    ):
        try:
            call()
        except dbus.exceptions.DBusException:
            pass
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
