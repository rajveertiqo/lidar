"""Build / extend the phrase registry (contract/vocab_v1.json) from the device's
closed vocabulary (utils/simple_guidance.ALLOWED).

PROTOTYPE tool. The registry is APPEND-ONLY: an id, once given to a sentence, is
never reused or renumbered. A sentence the device no longer says is marked
"retired"; a new one gets the next free id and the revision goes up. An app built
against an older revision still works — it falls back to ALERT_TEXT for any id it
does not know.

    venv/bin/python -m ble_bridge.make_vocab            # write/extend the registry
    venv/bin/python -m ble_bridge.make_vocab --check    # exit 1 if it is out of date
"""

import argparse
import hashlib
import json
import os
import sys
from typing import Any, Dict, List

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
VOCAB_PATH = os.path.join(HERE, "contract", "vocab_v1.json")


def _describe(sg) -> List[Dict[str, Any]]:
    """Each allowed sentence with the fields guidance() reports for it."""
    out = []
    side_left = (
        sg.DRIFT_LEFT,
        sg.NEAR_LEFT,
        sg.VEHICLE_FROM_LEFT,
        sg.GLASS_LEFT,
        sg.TURN_LEFT,
    )
    side_right = (
        sg.DRIFT_RIGHT,
        sg.NEAR_RIGHT,
        sg.VEHICLE_FROM_RIGHT,
        sg.GLASS_RIGHT,
        sg.TURN_RIGHT,
    )
    for text in sg.ALLOWED:
        level = sg._LEVEL[text]
        if text in sg._STEER_INFO:
            side = sg._STEER_INFO[text]["turn"]
        elif text in sg._MOVER_INFO:
            side = sg._MOVER_INFO[text]["move"]
        elif text in side_left:
            side = "left"
        elif text in side_right:
            side = "right"
        else:
            side = None
        if text in sg._VEHICLE_SENTENCES:
            obstacle = "vehicle"
        elif text in sg._MOVER_INFO:
            obstacle = sg._MOVER_INFO[text]["word"].lower()
        elif text in sg._STEER_INFO:
            obstacle = (sg._STEER_INFO[text]["word"] or "").lower() or None
        elif text in (sg.BLOCKED_GLASS, sg.GLASS_AHEAD) + sg._GLASS_BESIDE:
            obstacle = "glass"
        elif text in (sg.BLOCKED_WALL, sg.WALL_AHEAD) + sg._SIDE_WALL:
            obstacle = "wall"
        elif text in sg._OBJECT_OF:
            obstacle = sg._OBJECT_OF[text].lower()
        else:
            obstacle = None
        # unified_guidance: urgent = rule in (blocked, vehicle_approaching, mover_approaching,
        # offline) — every level-2 and level-3 sentence. "Guidance stopped" too, since it cuts
        # in (2026-10-23); this said otherwise until 2026-10-30, so the phone did not.
        urgent = level >= 2
        out.append(
            {
                "text": text,
                "action": sg._ACTION.get(text, "unknown"),
                "level": level,
                "urgent": urgent,
                "side": side,
                "obstacle": obstacle,
                "mover": text in sg._MOVER_INFO,
            }
        )
    return out


def build(existing: Dict[str, Any], current: List[Dict[str, Any]]) -> Dict[str, Any]:
    phrases = {p["id"]: dict(p) for p in existing.get("phrases", [])}
    by_text = {p["text"]: pid for pid, p in phrases.items()}
    changed = False
    next_id = max(phrases, default=0) + 1
    live = set()
    for entry in current:
        pid = by_text.get(entry["text"])
        if pid is None:
            pid, next_id = next_id, next_id + 1
            phrases[pid] = {"id": pid, **entry}
            changed = True
        else:
            fresh = {"id": pid, **entry}
            if {k: phrases[pid].get(k) for k in fresh} != fresh or phrases[pid].get(
                "retired"
            ):
                phrases[pid] = fresh
                changed = True
        live.add(pid)
    for pid, p in phrases.items():
        if pid not in live and not p.get("retired"):
            p["retired"] = True
            changed = True
    ordered = [phrases[k] for k in sorted(phrases)]
    digest = hashlib.sha256(
        json.dumps([[p["id"], p["text"]] for p in ordered], ensure_ascii=False).encode()
    ).hexdigest()[:16]
    revision = existing.get("revision", 0) + (1 if changed or not existing else 0)
    return {
        "registry": "shappe-phrases",
        "version": 1,
        "revision": revision,
        "sha": digest,
        "note": (
            "Append-only. id 0 = no sentence; 65535 = not in this registry (use "
            "ALERT_TEXT). Generated from utils/simple_guidance.ALLOWED by "
            "ble_bridge/make_vocab.py — do not edit by hand."
        ),
        "phrases": ordered,
    }, changed


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--check", action="store_true", help="exit 1 if the registry is out of date"
    )
    args = ap.parse_args()
    sys.path.insert(0, REPO)
    from utils import (
        simple_guidance as sg,
    )  # import only: starts no thread, opens no device

    existing = {}
    if os.path.exists(VOCAB_PATH):
        with open(VOCAB_PATH, encoding="utf-8") as fh:
            existing = json.load(fh)
    reg, changed = build(existing, _describe(sg))
    if args.check:
        print(
            "up to date"
            if not changed
            else "OUT OF DATE: run python -m ble_bridge.make_vocab"
        )
        return 1 if changed else 0
    if changed or not existing:
        os.makedirs(os.path.dirname(VOCAB_PATH), exist_ok=True)
        tmp = VOCAB_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(reg, fh, indent=1, ensure_ascii=False)
            fh.write("\n")
        os.replace(tmp, VOCAB_PATH)
    live = sum(1 for p in reg["phrases"] if not p.get("retired"))
    print(
        f"revision {reg['revision']}, {live} live phrases, sha {reg['sha']}"
        f"{' (written)' if changed or not existing else ' (unchanged)'}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
