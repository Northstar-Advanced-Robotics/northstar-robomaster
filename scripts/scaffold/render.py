"""Template rendering, file emission and clang-format integration.

Only ever creates new files. Nothing in here modifies a file that already
exists unless --force was passed and the path is one of our own targets.
"""

import os
import re
import string
import subprocess
from pathlib import Path

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"


class RenderError(Exception):
    pass


def _load(name):
    path = TEMPLATE_DIR / name
    if not path.is_file():
        raise RenderError("missing template: %s" % path)
    return string.Template(path.read_text())


def render(template_name, values):
    try:
        return _load(template_name).substitute(values)
    except KeyError as exc:
        raise RenderError(
            "template %s references unknown placeholder %s" % (template_name, exc)
        )


def find_class(src_root, class_name):
    """Locate a class under src/ and return its include path and namespace.

    A definition (`class X : public ...` / `class X {`) always beats a forward
    declaration (`class X;`). That matters: `ChassisSubsystem` is forward
    declared in seven command headers and defined in exactly one place, and
    resolving --requires to a command header would emit a bad include.
    """
    pattern = re.compile(r"^[ \t]*class\s+%s\b[ \t]*(;?)" % re.escape(class_name), re.M)
    ns_decl = re.compile(r"^namespace\s+([A-Za-z_][\w:]*)\s*$", re.M)
    preferred_stem = _snake(class_name)

    best = None
    for header in sorted(Path(src_root).rglob("*.hpp")):
        try:
            text = header.read_text(errors="replace")
        except OSError:
            continue
        for match in pattern.finditer(text):
            is_forward = match.group(1) == ";"
            score = (0 if is_forward else 2) + (1 if header.stem == preferred_stem else 0)
            if best and score <= best["score"]:
                continue
            # The innermost `namespace x::y` opened before the declaration wins.
            namespace = None
            for ns in ns_decl.finditer(text[: match.start()]):
                if ns.group(1) != "tap":  # skip `namespace tap { class Drivers; }` stubs
                    namespace = ns.group(1)
            best = {
                "include": header.relative_to(src_root).as_posix(),
                "namespace": namespace,
                "path": header,
                "score": score,
                "definition": not is_forward,
            }
    return best


def _snake(class_name):
    """CamelCase -> snake_case, matching this repo's file naming."""
    tokens = re.findall(r"[A-Z]+(?![a-z])|[A-Z][a-z0-9]*|[a-z0-9]+", class_name)
    return "_".join(t.lower() for t in tokens)


def class_exists(src_root, class_name):
    return find_class(src_root, class_name) is not None


def write_files(files, dry_run=False, force=False):
    """`files` is a list of (Path, text). Returns a list of result dicts."""
    results = []
    for path, text in files:
        existed = path.exists()
        if existed and not force:
            raise RenderError(
                "%s already exists (pass --force to overwrite)" % path
            )
        if not dry_run:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        results.append({"path": str(path), "existed": existed, "written": not dry_run})
    return results


def clang_format(paths, style_root):
    """Format the given files in place, then verify they are gate-clean.

    Mirrors what `.gitlab-ci.yml` checks: a non-empty `<replacement` list means
    the file would fail CI. Missing clang-format is a warning, never fatal.
    """
    paths = [str(p) for p in paths]
    if not paths:
        return {"available": True, "clean": True, "checked": [], "note": None}

    try:
        subprocess.run(
            ["clang-format", "-i", "--style=file"] + paths,
            cwd=str(style_root),
            check=True,
            capture_output=True,
        )
        check = subprocess.run(
            ["clang-format", "--output-replacements-xml", "--style=file"] + paths,
            cwd=str(style_root),
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return {
            "available": False,
            "clean": None,
            "checked": paths,
            "note": "clang-format not found on PATH; generated files were not formatted",
        }
    except subprocess.CalledProcessError as exc:
        return {
            "available": True,
            "clean": None,
            "checked": paths,
            "note": "clang-format failed: %s" % (exc.stderr or b"").strip(),
        }

    clean = "<replacement " not in check.stdout
    return {
        "available": True,
        "clean": clean,
        "checked": paths,
        "note": None if clean else "generated files still differ from .clang-format",
    }


def dominant_namespace(directory):
    """The namespace already used by headers in `directory`, if any.

    This repo is inconsistent -- `src/control/chassis/` uses `src::chassis`
    while `src/control/buzzer/` uses `src::control::buzzer`. Joining whatever a
    directory already does beats introducing a second namespace beside it.
    Returns None for a new or empty directory.
    """
    directory = Path(directory)
    if not directory.is_dir():
        return None
    ns_decl = re.compile(r"^namespace\s+([A-Za-z_][\w:]*)\s*$", re.M)
    counts = {}
    for header in list(directory.glob("*.hpp")) + list(directory.glob("*.cpp")):
        try:
            text = header.read_text(errors="replace")
        except OSError:
            continue
        for match in ns_decl.finditer(text):
            ns = match.group(1)
            if ns == "tap" or not ns.startswith("src"):
                continue
            counts[ns] = counts.get(ns, 0) + 1
    if not counts:
        return None
    return max(sorted(counts), key=lambda k: counts[k])
