"""Build manual/snoop37-manual.html from manual/src.html.
Puts in the firmware's own font (taken from web/editor.html) and the screenshots as data: URIs.
  python -I manual/build.py"""
import base64
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
src = (ROOT / "manual" / "src.html").read_text(encoding="utf-8")
editor = (ROOT / "web" / "editor.html").read_text(encoding="utf-8")
font = re.search(r"/\*SLOOP-FONT\*/(url\(data:font/ttf;base64,[A-Za-z0-9+/=]+\))", editor).group(1)
src = src.replace("/*FONT*/", font)


def img(m):
    p = ROOT / m.group(1)
    kind = {"png": "image/png", "webp": "image/webp"}[p.suffix[1:]]
    return "data:%s;base64,%s" % (kind, base64.b64encode(p.read_bytes()).decode())


src = re.sub(r"\{\{IMG:([^}]+)\}\}", img, src)
out = ROOT / "manual" / "snoop37-manual.html"
out.write_text(src, encoding="utf-8")
print(out, round(out.stat().st_size / 1024), "KB")
