"""label_report --decide: when a not-yet-spoken feature is told to speak. Run with
venv/bin/python -m unittest tests.test_label_report."""

import unittest

from tools.field import label_report as lr
from tools.field.walk_replay import FEATURES


def rows(spec):
    """[(label, class named), ...] -> labels_in rows, one clip each."""
    return [
        {"file": f"clip-{i}.jsonl.gz", "t": float(i), "label": label,
         "now": {"instruction": "", "class": named}}
        for i, (label, named) in enumerate(spec)
    ]


REST = [("wall", "wall")] * 15 + [("nothing", "nothing")] * 10  # 25 others, all right


class DecideTests(unittest.TestCase):
    def test_every_feature_it_decides_can_be_replayed(self):
        for feature in lr.DECIDE:
            self.assertIn(feature, FEATURES)

    def test_turns_on_what_names_its_labels_and_costs_nothing(self):
        base = rows([("stairs", "nothing")] * 12 + REST)
        on = rows([("stairs", "stairs")] * 11 + [("stairs", "nothing")] + REST)
        res = lr.judge(base, on, ("stairs", "kerb"))
        self.assertEqual(res["verdict"], "turn on", res["why"])
        self.assertEqual((res["right_off"], res["right_on"]), (0, 11))

    def test_keeps_off_what_takes_walls_for_stairs(self):
        base = rows([("stairs", "nothing")] * 12 + REST)
        on_rest = [("wall", "stairs")] * 3 + REST[3:]  # 3 of 25 right labels made wrong: 12%
        on = rows([("stairs", "stairs")] * 12 + on_rest)
        res = lr.judge(base, on, ("stairs", "kerb"))
        self.assertEqual(res["verdict"], "keep off")
        self.assertEqual(len(res["broken"]), 3)
        self.assertIn("other right labels wrong", res["why"])

    def test_keeps_off_what_names_too_few(self):
        base = rows([("glass", "wall")] * 10 + REST)
        on = rows([("glass", "glass")] * 5 + [("glass", "wall")] * 5 + REST)
        res = lr.judge(base, on, ("glass",))
        self.assertEqual(res["verdict"], "keep off")
        self.assertIn("names only 5 of 10", res["why"])

    def test_keeps_off_what_adds_nothing(self):
        base = rows([("person", "person")] * 10 + REST)
        res = lr.judge(base, rows([("person", "person")] * 10 + REST), ("person",))
        self.assertEqual(res["verdict"], "keep off")
        self.assertIn("nothing more", res["why"])

    def test_asks_for_more_labels_before_judging(self):
        base = rows([("glass", "wall")] * 4 + REST[:8])
        on = rows([("glass", "glass")] * 4 + REST[:8])
        res = lr.judge(base, on, ("glass",))
        self.assertEqual(res["verdict"], "more labels")
        self.assertIn("6 more glass", res["why"])
        self.assertIn("12 more of the other labels", res["why"])


if __name__ == "__main__":
    unittest.main()
