"""Which segmentation classes are walkable ground — the segmentation view draws them bright."""

WALKABLE_IDS: tuple = ()

WALKABLE_NAMES = frozenset({
    "path", "hallway", "corridor", "doorway",
    "floor", "floo", "floos", "woodfloor", "floor tile", "floor marble",
    "tile floor", "ground", "pavement", "sidewalk",
    "carpet", "floor carpet", "rug", "wall to wall carpet",
})

HAZARD_NAMES = frozenset({
    # traffic: a road or a crossing is never walkable ground (training/ adds these classes)
    "road", "street", "crosswalk",
    # fall hazards
    "stairs", "staircase", "spiral staircase", "step", "steps",
    "escalator", "escalator crop", "ladder", "platform",
    # things that are in the way, not underfoot
    "person", "people", "pedestrian", "rider", "cyclist",
    "bicycle", "motorcycle", "scooter", "car", "truck", "bus", "train",
})


def resolve_walkable_ids(class_names, default=WALKABLE_IDS):
    """Map a {id: name} class table to the IDs that count as walkable ground."""
    ids = sorted(
        cid for cid, name in (class_names or {}).items()
        if (n := str(name).strip().lower()) in WALKABLE_NAMES and n not in HAZARD_NAMES
    )
    if ids:
        return tuple(ids)
    if class_names:
        print("[Seg] WARNING: no class in this model matches a known walkable "
              f"surface {sorted(WALKABLE_NAMES)[:6]}...; got "
              f"{sorted(str(n) for n in class_names.values())[:12]}. "
              "Nothing will be drawn as walkable — add the name to "
              "walkable_guidance.WALKABLE_NAMES.")
    return tuple(default)
