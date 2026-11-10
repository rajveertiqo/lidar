"""The web pages' scripts and styles are intact. Run with
venv/bin/python -m unittest tests.test_templates.

An editor once dropped a "🔊" into the /map page's code (`new THREE.Mesh(🔊`, 2026-10-30): the
whole 3D script failed to load and nothing said so. Two checks: every inline script parses
(`node --check`, when node is installed), and — without node — no character outside ASCII
stands in actual code, outside strings, comments and regular expressions."""

import glob
import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PAGES = sorted(glob.glob(str(ROOT / "templates" / "*.html")))
_BLOCK = re.compile(r"<(script|style)([^>]*)>(.*?)</\1>", re.S | re.I)
_REGEX_BEFORE = set("(,=:[!&|?{};+-*%<>~^") | {""}


def blocks(page):
    """(kind, attributes, body, line it starts on) for each inline script and style."""
    html = Path(page).read_text(encoding="utf-8")
    for m in _BLOCK.finditer(html):
        kind, attrs, body = m.group(1).lower(), m.group(2), m.group(3)
        if kind == "script" and re.search(r"\bsrc\s*=", attrs):
            continue
        yield kind, attrs, body, html.count("\n", 0, m.start(3)) + 1


def stray_in_code(body, js=True):
    """(offset, character) of every non-ASCII character outside strings and comments
    (and, in a script, regular expressions). A template string's ${...} is code again, and
    may hold template strings of its own."""
    out, i, n, last = [], 0, len(body), ""
    depth, holes = 0, []  # brace depth; the depth each open ${ hole began at

    def template_text(j):
        """Skip template text from j: (index after it, whether it opened a ${ hole)."""
        while j < n:
            if body[j] == "\\":
                j += 2
            elif body[j] == "`":
                return j + 1, False
            elif body[j] == "$" and j + 1 < n and body[j + 1] == "{":
                return j + 2, True
            else:
                j += 1
        return n, False

    while i < n:
        ch = body[i]
        nxt = body[i + 1] if i + 1 < n else ""
        closes_hole = js and ch == "}" and holes and depth - 1 == holes[-1]
        if js and (ch == "`" or closes_hole):
            if closes_hole:
                holes.pop()
                depth -= 1
            i, hole = template_text(i + 1)
            if hole:
                holes.append(depth)
                depth += 1
            last = "{" if hole else "a"
            continue
        if ch in "\"'":
            j = i + 1
            while j < n and body[j] not in (ch, "\n"):
                j += 2 if body[j] == "\\" else 1
            i, last = j + 1, "a"
            continue
        if ch == "/" and nxt == "*":
            end = body.find("*/", i + 2)
            i = n if end < 0 else end + 2
            continue
        if js and ch == "/" and nxt == "/":
            end = body.find("\n", i)
            i = n if end < 0 else end
            continue
        if js and ch == "/" and last in _REGEX_BEFORE:  # a regular expression literal
            j, in_class = i + 1, False
            while j < n and body[j] != "\n" and (in_class or body[j] != "/"):
                if body[j] == "\\":
                    j += 1
                elif body[j] == "[":
                    in_class = True
                elif body[j] == "]":
                    in_class = False
                j += 1
            i, last = j + 1, "a"
            continue
        if js and ch == "{":
            depth += 1
        elif js and ch == "}":
            depth -= 1
        if ord(ch) > 127:
            out.append((i, ch))
        if not ch.isspace():
            last = ch if not (ch.isalnum() or ch in "_$") else "a"
        i += 1
    return out


def _node():
    found = shutil.which("node")
    if found:
        return found
    editors = sorted(glob.glob(os.path.expanduser("~/.vscode-server/bin/*/node")))
    return editors[-1] if editors else None


class TemplateTests(unittest.TestCase):
    def test_no_stray_characters_in_code(self):
        for page in PAGES:
            for kind, attrs, body, line in blocks(page):
                if "importmap" in attrs or "json" in attrs:
                    continue
                for offset, ch in stray_in_code(body, js=kind == "script"):
                    where = line + body.count("\n", 0, offset)
                    with self.subTest(page=os.path.basename(page), line=where):
                        self.fail(f"{ch!r} in the {kind}'s code at line {where}: "
                                  f"{body[max(0, offset - 30):offset + 10]!r}")

    def test_scripts_parse(self):
        node = _node()
        if not node:
            self.skipTest("node is not installed: only the stray-character check ran")
        with tempfile.TemporaryDirectory() as tmp:
            for page in PAGES:
                for k, (kind, attrs, body, line) in enumerate(blocks(page)):
                    if kind != "script":
                        continue
                    name = f"{os.path.basename(page)} script at line {line}"
                    with self.subTest(script=name):
                        if "importmap" in attrs or "json" in attrs:
                            json.loads(body)
                            continue
                        ext = ".mjs" if "module" in attrs else ".js"
                        path = os.path.join(tmp, f"{k}{ext}")
                        Path(path).write_text(body, encoding="utf-8")
                        run = subprocess.run([node, "--check", path], capture_output=True, text=True)
                        self.assertEqual(run.returncode, 0, f"{name}: {run.stderr.strip()[-400:]}")


if __name__ == "__main__":
    unittest.main()
