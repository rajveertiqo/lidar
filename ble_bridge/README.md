# `ble_bridge/` — Shappe BLE bridge (PROTOTYPE)

> **Status: PROTOTYPE, not production.**
>
> - **Tested:** protocol and bridge logic, by `test_ble_bridge.py` (49 desk tests).
> - **Checked:** BlueZ access, with `--check` (read-only).
> - **Not yet done:** it has **not** advertised or served a phone.
>
> The adapter on this Pi is rfkill soft-blocked. The first radio run must be supervised; see
> [docs/04 §8](../docs/04-bluetooth-communication-architecture.md#8-raspberry-pi-ble-implementation-guide).
> Nobody may wear the device while this is being developed.

The bridge is a **separate process**. It reads the navigation service's loopback HTTP API and serves it as a BLE GATT peripheral, so a phone app can do three things:
- show and speak the device's announcements;
- show the device's health;
- send a small set of harmless commands.

It changes **nothing** in the navigation service. The navigation service does not know the bridge exists, and a bridge crash cannot affect guidance.

| File | What it is |
|---|---|
| `protocol.py` | Reference implementation of the wire format: frames, fragmentation, the 14-byte ALERT and STATE records, JSON messages, and UUIDs. Must agree with [docs/06 (ICD)](../docs/06-interface-control-document.md). |
| `core.py` | `Bridge`: snapshots in, protocol messages out; commands in, responses out. Has no Bluetooth or network code, so it is fully testable. |
| `sources.py` | `HttpSource` (the live service on `127.0.0.1:8000`) and `MockSource` (scripted scenarios with no sensors). |
| `gatt_server.py` | BlueZ D-Bus front end: GATT application, advertisement, pairing agent and the poll thread. |
| `make_vocab.py` | Builds the append-only phrase registry `contract/vocab_v1.json` from `utils/simple_guidance.ALLOWED`. |
| `make_test_vectors.py` | Writes `contract/test_vectors.json`: golden frames that any mobile decoder must reproduce. |
| `test_ble_bridge.py` | Desk tests (`unittest`). |
| `shappe-ble.service` | systemd **user** unit. **Not installed.** |
| `contract/` | Machine-readable contract: phrase registry, test vectors and JSON Schemas. |

```bash
venv/bin/python -m unittest ble_bridge.test_ble_bridge -v     # desk tests
venv/bin/python -m ble_bridge.make_vocab --check              # registry still matches the device?
venv/bin/python -m ble_bridge.gatt_server --check             # BlueZ + adapter, read-only
# --- radio on: supervised only ---
sudo rfkill unblock bluetooth
venv/bin/python -m ble_bridge.gatt_server --mock sidewalk     # advertises "Shappe-M"
venv/bin/python -m ble_bridge.gatt_server                     # live: needs shappe-guide running
```

Mock mode advertises **`Shappe-M`** (a legacy advertisement fits at most 8 name bytes), and its Device Information Model Number is `Shappe Guide MOCK`, readable without pairing. `--mock-audio off` and `--mock-device-voice` exercise the two "app must stay silent" paths. It also sets `mock: true` in PROTOCOL_INFO and the `simulated_input` health bit, so an app can never present a mock as a real device.

## Known gaps (tracked in docs/09)

- **Pairing:** pairing is LE Secure Connections **Just Works**. That gives encryption but no MITM protection, and is acceptable only for the supervised prototype. The production design is in docs/04 §6.
- **One phone:** v1 serves one phone. BlueZ sends notifications to every subscribed central, and the per-connection sequence counters assume one.
- **MTU:** frames are cut for the smaller of the app's `hello` (`att_mtu`) and BlueZ's write `mtu` option, whichever are known. Until then, frames are cut for the 23-byte minimum.
- **iPhone pairing prompt:** an iPhone's automatic pairing prompt on ATT *Insufficient Encryption* (0x0F) still has to be confirmed on real iPhones (docs/04 §6.4).
