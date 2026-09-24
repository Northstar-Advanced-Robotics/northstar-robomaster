#!/usr/bin/env python3
"""Generate one navigation page per @defgroup, from Doxygen's XML.

Each page carries the group's own description and links to the canonical Exhale page for every
class in it. Links rather than embedded content, so a class is declared exactly once in the Sphinx
project -- embedding it here as well produces ~1000 "Duplicate C++ declaration" warnings.

Run after Doxygen and before sphinx-build; the Makefile's `html` target does both.
"""
import os
import re
import xml.etree.ElementTree as ET

XML_DIR = "doxyoutput/xml"
OUT_DIR = "subsystems"

# Order controls the sidebar order.
ORDER = [
    "chassis", "turret", "agitator", "flywheel", "hopper_kicker",
    "governors", "client_display", "communication", "robots", "util",
]


def text_of(node):
    """Flatten a Doxygen description node to plain text."""
    if node is None:
        return ""
    return re.sub(r"\s+", " ", "".join(node.itertext())).strip()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    written = []
    for name in ORDER:
        # Doxygen escapes underscores in filenames, so hopper_kicker -> group__hopper__kicker.xml
        path = os.path.join(XML_DIR, "group__" + name.replace("_", "__") + ".xml")
        if not os.path.exists(path):
            print(f"  skip {name}: no {path}")
            continue
        compound = ET.parse(path).getroot().find("compounddef")
        title = text_of(compound.find("title")) or name
        brief = text_of(compound.find("briefdescription"))
        detail = text_of(compound.find("detaileddescription"))

        classes = []
        for inner in compound.findall("innerclass"):
            refid = inner.get("refid")
            display = (inner.text or "").strip()
            display = display.split("::")[-1] if display else refid
            classes.append((display, refid))
        classes.sort(key=lambda c: c[0].lower())

        lines = ["=" * len(title), title, "=" * len(title), ""]
        if brief:
            lines += [brief, ""]
        if detail and detail != brief:
            lines += [detail, ""]
        if classes:
            lines += [f"{len(classes)} classes:", ""]
            for display, refid in classes:
                lines.append(f"* :doc:`{display} <../api/{refid}>`")
            lines.append("")
        else:
            lines += ["Nothing is currently tagged into this group.", ""]

        with open(os.path.join(OUT_DIR, f"{name}.rst"), "w") as f:
            f.write("\n".join(lines))
        written.append((name, len(classes)))

    for name, n in written:
        print(f"  {name:16} {n} classes")
    print(f"generated {len(written)} subsystem pages")


if __name__ == "__main__":
    main()
