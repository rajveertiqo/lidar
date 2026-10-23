"""Build the consolidated master document from docs/README.md, 01-10 and references.md.

    python3 docs/build_master.py            # MASTER.md + MASTER.html
    python3 docs/build_master.py --pdf      # ... + MASTER.pdf (headless Chromium; Mermaid from jsDelivr)

MASTER.md is plain concatenation with links between documents rewritten to in-page
anchors. MASTER.html renders Markdown (python-markdown) and Mermaid (in the browser),
and prints cleanly to A4.
"""

import argparse
import html
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER = [
    "README.md",
    "01-project-overview.md",
    "02-hardware-architecture.md",
    "03-perception-and-navigation-pipeline.md",
    "04-bluetooth-communication-architecture.md",
    "05-mobile-developer-integration-guide.md",
    "06-interface-control-document.md",
    "07-user-journeys-and-accessibility.md",
    "08-safety-reliability-failure-management.md",
    "09-roadmap-and-integration-checklist.md",
    "10-architecture-and-sequence-diagrams.md",
    "references.md",
]
MERMAID_CDN = "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.min.js"


def anchor(name: str) -> str:
    return "doc-" + os.path.splitext(name)[0].lower().replace("readme", "00-index")


def rewrite_links(text: str) -> str:
    names = "|".join(re.escape(n) for n in ORDER)
    # (02-hardware-architecture.md#x) or (02-hardware-architecture.md) -> (#doc-02-...)
    text = re.sub(
        rf"\(({names})(#[^)]*)?\)", lambda m: f"(#{anchor(m.group(1))})", text
    )
    # links out of docs/ (../ble_bridge/...) stay relative to docs/; fine for MASTER.* beside them
    return text


def build_md() -> str:
    parts = []
    for name in ORDER:
        with open(os.path.join(HERE, name), encoding="utf-8") as fh:
            body = rewrite_links(fh.read())
        parts.append(f'<a id="{anchor(name)}"></a>\n\n{body.strip()}\n')
    return '\n\n<div class="pagebreak"></div>\n\n'.join(parts)


CSS = """
:root { --fg:#1b1f24; --muted:#57606a; --bg:#ffffff; --line:#d0d7de; --code:#f6f8fa; --accent:#0b5cad; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --fg:#e6edf3; --muted:#9aa4ae; --bg:#0d1117; --line:#30363d; --code:#161b22; --accent:#58a6ff; } }
html { background: var(--bg); }
body { font: 15px/1.55 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; color: var(--fg);
       background: var(--bg); max-width: 1000px; margin: 0 auto; padding: 24px 16px 64px; }
h1 { font-size: 1.7em; border-bottom: 2px solid var(--line); padding-bottom: .3em; margin-top: 1.6em; }
h2 { font-size: 1.3em; border-bottom: 1px solid var(--line); padding-bottom: .2em; margin-top: 1.5em; }
h3 { font-size: 1.1em; }
a { color: var(--accent); }
table { border-collapse: collapse; width: 100%; margin: .8em 0; font-size: .86em; display: block; overflow-x: auto; }
th, td { border: 1px solid var(--line); padding: 5px 7px; vertical-align: top; text-align: left; }
th { background: var(--code); }
code { background: var(--code); padding: .1em .3em; border-radius: 4px; font-size: .9em; }
pre { background: var(--code); padding: 10px 12px; border-radius: 6px; overflow-x: auto; font-size: .82em; }
pre code { background: none; padding: 0; }
pre.mermaid { background: transparent; text-align: center; }
blockquote { border-left: 4px solid var(--accent); margin: 1em 0; padding: .2em 1em; color: var(--muted); }
.cover { text-align: center; padding: 18vh 0 10vh; }
.cover h1 { border: 0; font-size: 2.2em; }
.cover p { color: var(--muted); }
@media print {
  body { max-width: none; padding: 0; font-size: 10.5pt; }
  .pagebreak { break-after: page; }
  table { display: table; font-size: 8.5pt; }
  tr, pre, pre.mermaid, svg { break-inside: avoid; }
  pre.mermaid svg { max-height: 225mm; }   /* a diagram taller than a page would print blank */
  h1, h2, h3 { break-after: avoid; }
  a { color: inherit; text-decoration: none; }
  @page { size: A4; margin: 16mm 14mm; }
}
"""


def build_html(md_text: str) -> str:
    import markdown

    blocks = []

    def stash(m):
        blocks.append(m.group(1))
        return f"\n\nMERMAIDBLOCK{len(blocks) - 1}END\n\n"

    text = re.sub(r"```mermaid\n(.*?)```", stash, md_text, flags=re.S)
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "sane_lists"])
    for i, src in enumerate(blocks):
        body = body.replace(
            f"<p>MERMAIDBLOCK{i}END</p>",
            f'<pre class="mermaid">{html.escape(src)}</pre>',
        )
    cover = (
        '<div class="cover"><h1>Shappe Guide: Engineering Documentation Package</h1>'
        "<p>AI-assisted obstacle-warning wearable for blind and visually impaired pedestrians<br>"
        "Hardware, perception pipeline, Bluetooth protocol and mobile integration</p>"
        "<p>Prepared 2026-10-09 from repository <code>~/Documents/lidar</code> (HEAD e209c48 + working tree)</p>"
        "<p>Status labels: CONFIRMED · HW-VERIFIED · PROPOSED · NOT IMPLEMENTED · NEEDS VALIDATION</p></div>"
        '<div class="pagebreak"></div>'
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shappe Guide Engineering Docs</title>
<style>{CSS}</style></head>
<body>{cover}{body}
<script src="{MERMAID_CDN}"></script>
<script>
  const dark = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
  mermaid.initialize({{ startOnLoad: false, securityLevel: "strict",
                        theme: dark ? "dark" : "default", flowchart: {{ htmlLabels: true }},
                        sequence: {{ wrap: true, width: 130, actorMargin: 30, messageMargin: 28,
                                     boxMargin: 6, mirrorActors: false, noteMargin: 8 }} }});
  mermaid.run({{ querySelector: "pre.mermaid" }}).then(() => {{ document.body.dataset.mermaid = "done"; }})
    .catch((e) => {{ document.body.dataset.mermaid = "error: " + e; }});
</script>
</body></html>
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--pdf",
        action="store_true",
        help="also print MASTER.pdf with headless Chromium",
    )
    args = ap.parse_args()
    md_text = build_md()
    with open(os.path.join(HERE, "MASTER.md"), "w", encoding="utf-8") as fh:
        fh.write(md_text)
    html_path = os.path.join(HERE, "MASTER.html")
    with open(html_path, "w", encoding="utf-8") as fh:
        fh.write(build_html(md_text))
    print(f"MASTER.md, MASTER.html written ({len(md_text) // 1024} KiB of Markdown)")
    if args.pdf:
        chrome = (
            shutil.which("chromium")
            or shutil.which("chromium-browser")
            or shutil.which("google-chrome")
        )
        if not chrome:
            print("no Chromium found; open MASTER.html and print to PDF")
            return 1
        out = os.path.join(HERE, "MASTER.pdf")
        subprocess.run(
            [
                chrome,
                "--headless=new",
                "--disable-gpu",
                "--no-pdf-header-footer",
                "--virtual-time-budget=60000",
                "--run-all-compositor-stages-before-draw",
                "--force-prefers-reduced-motion",
                "--blink-settings=preferredColorScheme=1",
                f"--print-to-pdf={out}",
                "file://" + html_path,
            ],
            check=True,
            timeout=300,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"MASTER.pdf written ({os.path.getsize(out) // 1024} KiB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
