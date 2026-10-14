# References

All accessed **2026-10-09**. Tags:
- **[DS]** datasheet
- **[OD]** official documentation
- **[SPEC]** specification or standard
- **[VM]** vendor marketing (not a validated system claim)
- **[3P]** third party

Source IDs `[Sx]` are cited in docs 02 and 08. Where sources disagree, the documents state the disagreement rather than picking one.

## Hardware

| ID | Title (version) | URL | Tag |
|---|---|---|---|
| S1 | Arducam, 16MP IMX519 Autofocus Synchronized Quad-Camera Kit (B0388) | https://www.arducam.com/blog/product/arducam-16mp-imx519-autofocus-synchronized-quad-camera-kit-for-raspberry-pi-nvidia-jetson/ | VM |
| S2 | Arducam Wiki, Quick Start Guide for Arducam CamArray HAT Kit | https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/quick-start/ | OD |
| S3 | Arducam Wiki, Multi-Camera CamArray | https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/Multi-Camera-CamArray/ | OD |
| S4 | Arducam Wiki, 16MP IMX519 | https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/16MP-IMX519/ | OD |
| S5 | Arducam, 16MP IMX519 Camera Module (80°D/67°H/53°V) | https://www.arducam.com/blog/?p=105181 | VM |
| S6 | Arducam forum, CamArray frame rate thread (vendor staff posts) | https://forum.arducam.com/t/why-cant-i-reach-120fps-using-4-imx519-cameras-with-camarray-hat/5368 | 3P |
| S7 | raspberrypi/linux `drivers/media/i2c/imx519.c` (rpi-6.12.y) | https://raw.githubusercontent.com/raspberrypi/linux/rpi-6.12.y/drivers/media/i2c/imx519.c | OD |
| S8 | Raspberry Pi documentation, Camera software | https://www.raspberrypi.com/documentation/computers/camera_software.html | OD |
| S9 | SLAMTEC, RPLIDAR A2 Introduction and Datasheet, model A2M12 (rev 1.0, 2022-04-08) | https://cdn.robotshop.com/media/r/rpk/rb-rpk-22/pdf/rplidar-a2m12-360-laser-range-scanner-datasheet.pdf | DS |
| S10 | SLAMTEC LR001, Interface Protocol and Application Notes (rev 2.2, 2021-02-25) | https://cdn.robotshop.com/media/r/rpk/rb-rpk-07/pdf/lr001_slamtec_rplidar_protocol_v2.2_en.pdf | OD |
| S11 | SLAMTEC, Common Indoor SLAM Failure Scenarios | https://www.slamtec.com/en/news/detail/855 | VM |
| S12 | Acconeer, A121 Pulsed Coherent Radar Datasheet (v1.8, 2025-02-03) | https://developer.acconeer.com/download/a121-datasheet | DS |
| S13 | Acconeer, XM125 Datasheet (v1.2, 2023-10-04) | https://developer.acconeer.com/download/xm125-datasheet | DS |
| S14 | Acconeer docs, Distance Detector (Exploration Tool 7.18.2) | https://docs.acconeer.com/en/latest/detectors/a121/distance_detector.html | OD |
| S15 | Acconeer docs, Presence Detector | https://docs.acconeer.com/en/latest/detectors/a121/presence_detector.html | OD |
| S16 | Acconeer docs, Speed Detector | https://docs.acconeer.com/en/latest/detectors/a121/speed_detector.html | OD |
| S17 | Acconeer docs, Exploration Tool changelog | https://docs.acconeer.com/en/latest/exploration_tool/changelog.html | OD |
| S18 | Waveshare Wiki, A121 Range Sensor (SKU 33777) | https://www.waveshare.com/wiki/A121_Range_Sensor | OD |
| S19 | Waveshare, A121 Range Sensor product page | https://www.waveshare.com/a121-range-sensor.htm | VM |
| S20 | WCH CH342 datasheet (summary only) | https://cdn.sparkfun.com/assets/8/e/f/5/1/CH342_Datasheet.pdf | 3P |
| S21 | Raspberry Pi 5 product page | https://www.raspberrypi.com/products/raspberry-pi-5/ | VM |
| S22 | Raspberry Pi documentation, Raspberry Pi computer hardware | https://www.raspberrypi.com/documentation/computers/raspberry-pi.html | OD |
| S23 | Raspberry Pi news, Introducing Raspberry Pi 5 (2023-09-28) | https://www.raspberrypi.com/news/introducing-raspberry-pi-5/ | VM |
| S24 | Raspberry Pi 27W USB-C Power Supply | https://www.raspberrypi.com/products/27w-power-supply/ | VM |
| S25 | Raspberry Pi Active Cooler | https://www.raspberrypi.com/products/active-cooler/ | VM |
| S26 | Raspberry Pi AI HAT+ 2 Product Brief (RP-009655-MM-6, July 2026) | https://pip-assets.raspberrypi.com/categories/1319-raspberry-pi-ai-hat-2/documents/RP-009655-MM-6-raspberry-pi-ai-hat-plus-2-product-brief.pdf | OD |
| S27 | Raspberry Pi documentation, AI HATs | https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html | OD |
| S28 | Raspberry Pi documentation, AI software | https://www.raspberrypi.com/documentation/computers/ai.html | OD |
| S29 | Raspberry Pi news, Introducing the AI HAT+ 2 | https://www.raspberrypi.com/news/introducing-the-raspberry-pi-ai-hat-plus-2-generative-ai-on-raspberry-pi-5/ | VM |
| S30 | Raspberry Pi news, When and why you might need the AI HAT+ 2 | https://www.raspberrypi.com/news/when-and-why-you-might-need-the-raspberry-pi-ai-hat-plus-2/ | VM |
| S31 | Hailo-10H product page | https://hailo.ai/products/ai-accelerators/hailo-10h-ai-accelerator/ | VM |
| S32 | RidgeRun wiki, Hailo Dataflow Compiler | https://developer.ridgerun.com/wiki/index.php/Hailo/Hailo-8/AI_Software_and_Tools/Hailo_Dataflow_Compiler | 3P |
| S33 | Raspberry Pi forum, AI HAT + NVMe thread | https://forums.raspberrypi.com/viewtopic.php?t=395552 | 3P |
| S34 | Intel, USB 3.0 RFI Impact on 2.4 GHz Wireless Devices (327216-001, Apr 2012) | https://www.usb.org/sites/default/files/327216.pdf | OD |
| S35 | TDK InvenSense MPU-6500 Product Specification (rev 1.0) | https://ocw.cs.pub.ro/courses/_media/pm/prj2017/amocanu/mpu_6500_rev1.0_1_.pdf | DS |
| S36 | TDK MPU-6500 product page | https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6500/ | VM |
| S37 | Infineon CYW43455 product page (specs not visible when fetched) | https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-802.11ac/cyw43455/ | VM |
| S38 | DigiKey listing, Arducam B0371-1 | https://www.digikey.com/en/products/detail/arducam/B0371-1/22598757 | 3P |

**Known contradictions:**
- **IMX519 HFOV:** 66° [S1, S4] vs 67° [S5]. A pinhole calculation gives 67.1°.
- **UC-512 quad frame rates:** [S1] vs [S3].
- **Radar FOV axes:** Waveshare [S18] vs Acconeer E/H planes [S12].
- **Waveshare "accuracy 0.03 m":** this is the radial resolution, not the accuracy [S12].
- **AI HAT+ 2 performance claims:** they differ across [S26], [S30].
- **I²C bus number for the CamArray:** 4/6 [S2] vs 11 in this code (measured).

## Bluetooth

| Title | URL | Tag |
|---|---|---|
| BlueZ 5.82 D-Bus API docs: GattManager, GattService, GattCharacteristic (Oct 2023), LEAdvertisingManager, LEAdvertisement, AgentManager, Agent | https://raw.githubusercontent.com/bluez/bluez/5.82/doc/ (canonical: https://git.kernel.org/pub/scm/bluetooth/bluez.git/tree/doc/) | OD |
| BlueZ `src/gatt-database.c`, `src/shared/gatt-server.c` (CCC auto-creation; 0x0F vs 0x05 errors) | https://github.com/bluez/bluez/tree/5.82/src | OD (source) |
| BlueZ `src/main.conf` and `doc/settings-storage.txt` (5.82) | https://github.com/bluez/bluez/blob/5.82/src/main.conf | OD |
| Debian trixie `bluez` 5.82-1.1 | https://packages.debian.org/trixie/bluez | OD |
| Bluetooth Core 5.4, Vol 3 Part F (ATT): default MTU 23, attribute value ≤ 512 B, indication confirmation | https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/attribute-protocol--att-.html | SPEC |
| Bluetooth Core 5.4, Vol 3 Part H (SMP): LE Secure Connections, association models | https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/security-manager-specification.html | SPEC |
| Bluetooth LE Primer v1.3.0 (2024-10-15) | https://www.bluetooth.com/bluetooth-le-primer/ | OD |
| Bluetooth SIG blog, Bluetooth Pairing Part 4 (2017-01-19) | https://www.bluetooth.com/blog/bluetooth-pairing-part-4/ | OD |
| Bluetooth Assigned Numbers (version date 2026-10-08) | https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf | SPEC |
| Apple, `scanForPeripherals(withServices:options:)` | https://developer.apple.com/documentation/corebluetooth/cbcentralmanager/scanforperipherals(withservices:options:) | OD |
| Apple, `connect(_:options:)` (connections do not time out) | https://developer.apple.com/documentation/corebluetooth/cbcentralmanager/connect(_:options:) | OD |
| Apple, Core Bluetooth Programming Guide: background processing (archive, 2013-09-18) | https://developer.apple.com/library/archive/documentation/NetworkingInternetWeb/Conceptual/CoreBluetooth_concepts/CoreBluetoothBackgroundProcessingForIOSApps/PerformingTasksWhileYourAppIsInTheBackground.html | OD |
| Apple TN3115, Bluetooth State Restoration app relaunch rules (rev 2025-09-15) | https://developer.apple.com/documentation/technotes/tn3115-bluetooth-state-restoration-app-relaunch-rules | OD |
| Apple, AccessorySetupKit | https://developer.apple.com/documentation/accessorysetupkit | OD |
| Apple QA1931, BLE advertising and connection parameters (2017) | https://developer.apple.com/library/archive/qa/qa1931/_index.html | OD |
| Apple, NSBluetoothAlwaysUsageDescription | https://developer.apple.com/documentation/bundleresources/information-property-list/nsbluetoothalwaysusagedescription | OD |
| Nordic DevZone, iOS pairing initiation (quotes Apple ADG §40.10) | https://devzone.nordicsemi.com/f/nordic-q-a/86794/how-to-initiate-ble-pairing-manually | 3P |
| Android, Bluetooth permissions | https://developer.android.com/develop/connectivity/bluetooth/bt-permissions | OD |
| Android 14 behaviour changes (MTU 517) | https://developer.android.com/about/versions/14/behavior-changes-all | OD |
| Android, Transfer BLE data (CCCD write) | https://developer.android.com/develop/connectivity/bluetooth/ble/transfer-ble-data | OD |
| Android, Connect to a GATT server | https://developer.android.com/develop/connectivity/bluetooth/ble/connect-gatt-server | OD |
| Android, BLE in the background | https://developer.android.com/develop/connectivity/bluetooth/ble/background | OD |
| Android, Companion device pairing | https://developer.android.com/develop/connectivity/bluetooth/companion-device-pairing | OD |
| Android, Foreground service types | https://developer.android.com/develop/background-work/services/fgs/service-types | OD |
| Punch Through, Android BLE guide (2024-04-17) | https://punchthrough.com/android-ble-guide/ | 3P |

## Mobile, accessibility and audio

| Title | URL | Tag |
|---|---|---|
| W3C WCAG 2.2 (Recommendation; dated version 12 Dec 2024) | https://www.w3.org/TR/WCAG22/ | SPEC |
| W3C, Guidance on Applying WCAG 2.2 to Mobile Applications (Group Draft Note, 6 May 2025) | https://www.w3.org/TR/wcag2mobile-22/ | OD |
| ETSI EN 301 549 (V3.2.1 2021-03; V4.1.1 2026-09-02) | https://www.etsi.org/deliver/etsi_en/301500_301599/301549/ | SPEC |
| Apple HIG, Accessibility | https://developer.apple.com/design/human-interface-guidelines/accessibility | OD |
| Apple HIG, VoiceOver | https://developer.apple.com/design/human-interface-guidelines/voiceover | OD |
| Apple, UIAccessibility.Notification.announcement | https://developer.apple.com/documentation/uikit/uiaccessibility/notification/announcement | OD |
| Android, Make apps more accessible (48 dp, contentDescription) | https://developer.android.com/guide/topics/ui/accessibility/apps | OD |
| Android 16 behaviour changes (announceForAccessibility deprecated) | https://developer.android.com/about/versions/16/behavior-changes-all | OD |
| Apple AVFAudio reference (AVAudioSession categories and options, voicePrompt mode, AVSpeechSynthesizer) | https://developer.apple.com/documentation/avfaudio | OD |
| Apple App Review Guidelines 2.5.4 | https://developer.apple.com/app-store/review/guidelines/ | OD |
| Android, Manage audio focus | https://developer.android.com/media/optimize/audio-focus | OD |
| Android, Haptics APIs | https://developer.android.com/develop/ui/views/haptics/haptics-apis | OD |
| flutter_blue_plus (2.3.14, FlutterBluePlus License v1.6) | https://pub.dev/packages/flutter_blue_plus | OD |
| flutter_reactive_ble (5.6.0, BSD-3) | https://pub.dev/packages/flutter_reactive_ble | OD |
| react-native-ble-plx (3.5.1; Apache-2.0 repo vs MIT npm) | https://github.com/dotintent/react-native-ble-plx | OD |
| react-native-ble-manager (12.5.3, Apache-2.0) | https://github.com/innoveit/react-native-ble-manager | OD |
| Nordic Android-BLE-Library (2.11.0) / Kotlin-BLE-Library (2.0 beta) | https://github.com/NordicSemiconductor | OD |
| Kable (0.45.0, Apache-2.0) | https://github.com/JuulLabs/kable | OD |
| nRF Connect for Mobile | https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile | OD |
| LightBlue | https://punchthrough.com/lightblue/ | OD |
| bless (0.3.0, MIT) | https://pypi.org/project/bless/ | OD |
| Bumble (Google) | https://google.github.io/bumble/ | OD |
| Android Emulator netsim / advanced networking | https://developer.android.com/studio/run/emulator-networking-advanced | OD |

## Safety, regulation and ethics (scope only)

| Title | URL / reference | Tag |
|---|---|---|
| ISO 14971:2019 Medical devices: application of risk management | ISO catalogue | SPEC |
| IEC 62366-1:2015+AMD1:2020 Usability engineering | https://webstore.iec.ch/publication/67220 | SPEC |
| IEC 62304:2006+A1:2015 Medical device software lifecycle (Ed 2 not published as of 2026-08) | IEC catalogue | SPEC |
| ISO 13485:2016 | ISO catalogue | SPEC |
| EU MDR 2017/745, Art 2(1) | EUR-Lex (fetch failed; quoted via a 3P mirror) | SPEC |
| US 21 CFR 886.5900 Electronic vision aid | https://www.law.cornell.edu/cfr/text/21/886.5900 | 3P mirror |
| ISO 9999:2022; ISO 23599:2019; ISO 21542:2021 | ISO catalogue | SPEC |
| IEC 62368-1:2023 | https://webstore.iec.ch/publication/69308 | SPEC |
| IEC 60825-1:2014 | IEC catalogue | SPEC |
| EU RED 2014/53/EU; Delegated Reg. 2022/30; EN 18031-1/-2/-3 | OJEU | SPEC |
| ETSI EN 300 328 V2.2.2 | https://www.etsi.org/deliver/etsi_en/300300_300399/300328/ | SPEC |
| UN Manual of Tests and Criteria §38.3; IEC 62133-2:2017+AMD1:2021 | https://webstore.iec.ch/publication/70017 | SPEC |
| EU Accessibility Act 2019/882 (applies from 28 Jun 2025) | https://commission.europa.eu/ (EAA page) | OD |
| WMA Declaration of Helsinki (rev. Oct 2024) | https://www.wma.net/ | OD |
| ISO 14155:2026 Clinical investigation of medical devices | ISO catalogue | SPEC |

## Project sources (CONFIRMED evidence)

- **Repository:** `~/Documents/lidar` at HEAD `e209c48`, plus the uncommitted working tree, inspected 2026-10-09.
- **On-device commands** (read-only; their outputs are quoted in 02 §2):
  - `lsusb`, `ls /dev/serial/by-id`, `bluetoothctl show`, `rfkill list`, `wpctl status`;
  - `vcgencmd measure_temp`, `get_throttled`, `pmic_read_adc`;
  - `hailortcli --version`, `dpkg -l`;
  - `/boot/firmware/config.txt`, `/usr/share/dbus-1/system.d/bluetooth.conf`.
- `docs/SHAPPE_GUIDE_AUDIT.md` (2026-09-29 safety audit) and `README.md`.
