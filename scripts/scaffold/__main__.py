#!/usr/bin/env python3
"""Scaffold a new Taproot subsystem or command.

Creates the .hpp/.cpp under northstar-robomaster-project/src/control/<dir>/,
formats them with clang-format, then prints an exact checklist for wiring them
into each robot's control file.

This tool NEVER modifies an existing file. The wiring step is read-only
inspection; you paste the snippets yourself. Undo is always `rm -r` on the
generated directory.

    python3 scripts/scaffold subsystem Flywheel
    python3 scripts/scaffold command SpinUp --requires FlywheelSubsystem
"""

import argparse
import json
import sys
from pathlib import Path

import names as names_mod
import render
import wiring

REPO_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = REPO_ROOT / "northstar-robomaster-project"
SRC_ROOT = PROJECT_ROOT / "src"


class ScaffoldError(Exception):
    pass


def _add_shared_args(p):
    p.add_argument("name", help='e.g. "Flywheel", "flywheel_subsystem", "SpinUp"')
    p.add_argument(
        "--dir",
        dest="directory",
        default=None,
        help="feature directory under src/control/ (default: derived from the name)",
    )
    p.add_argument(
        "--robots",
        default="standard",
        help="comma-separated: %s -- or 'all', or 'none' to skip the wiring "
        "checklist" % ",".join(wiring.ROBOTS),
    )
    p.add_argument("--class", dest="class_override", default=None,
                   help="override the derived class name (e.g. for acronyms)")
    p.add_argument("--instance", dest="instance_override", default=None,
                   help="override the object name used in the wiring snippets")
    p.add_argument("--namespace", default=None,
                   help="override the namespace (default: match the directory, "
                        "else src::control::<dir>)")
    p.add_argument("--header-only", action="store_true", help="no .cpp file")
    p.add_argument("--dry-run", action="store_true", help="print what would happen; write nothing")
    p.add_argument("--json", dest="as_json", action="store_true", help="machine-readable output")
    p.add_argument("--force", action="store_true", help="overwrite existing files")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="scaffold",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="kind", required=True)

    _add_shared_args(sub.add_parser("subsystem", help="create a tap::control::Subsystem"))

    cmd = sub.add_parser("command", help="create a tap::control::Command")
    _add_shared_args(cmd)
    cmd.add_argument(
        "--requires",
        required=True,
        help="the subsystem class this command requires, e.g. FlywheelSubsystem",
    )
    cmd.add_argument(
        "--requires-include",
        default=None,
        help="override the include path for --requires (default: found by searching src/)",
    )
    return parser.parse_args(argv)


def resolve_robots(spec):
    """Parse --robots into a de-duplicated list in canonical order.

    Accepts a comma-separated list, the keyword `all`, or `none`. Whitespace
    around names is tolerated so `--robots "standard, hero"` works from a shell
    or a VS Code promptString.
    """
    spec = (spec or "").strip().lower()
    if spec in ("", "none"):
        return []
    if spec == "all":
        return list(wiring.ROBOTS)

    picked = [r.strip() for r in spec.split(",") if r.strip()]
    if "all" in picked:
        return list(wiring.ROBOTS)

    unknown = [r for r in picked if r not in wiring.ROBOTS]
    if unknown:
        raise ScaffoldError(
            "unknown robot(s) %s -- choose from %s, or use 'all' / 'none'"
            % (", ".join(unknown), ", ".join(wiring.ROBOTS))
        )
    # Canonical order, de-duplicated, so `hero,standard,hero` behaves sanely.
    return [r for r in wiring.ROBOTS if r in set(picked)]


def resolve_requires(args, target_dir, command_namespace):
    """Work out how the command should refer to the subsystem it requires."""
    cls = args.requires
    found = render.find_class(SRC_ROOT, cls)

    if args.requires_include:
        include = args.requires_include
    elif found:
        include = found["include"]
    else:
        raise ScaffoldError(
            "could not find `class %s` under src/. Pass --requires-include "
            "<path relative to src/> to say where it lives." % cls
        )

    # Same directory -> plain filename, matching play_song_command.hpp.
    if found and found["path"].parent == target_dir:
        include = found["path"].name

    qualified = cls
    if found and found["namespace"] and found["namespace"] != command_namespace:
        qualified = "%s::%s" % (found["namespace"], cls)

    tokens = names_mod.tokenize(cls)
    if len(tokens) > 1 and tokens[-1] == "subsystem":
        tokens = tokens[:-1]
    instance = names_mod.lower_camel(tokens)

    return {
        "requires_class": qualified,
        "requires_include": include,
        "requires_instance": instance,
        "found": bool(found),
    }


def build(args):
    kind = args.kind

    directory = args.directory.strip("/") if args.directory else None

    if directory is None and kind == "command":
        # A command belongs beside the subsystem it drives, not in a directory
        # named after itself: `command SpinUp --requires FlywheelSubsystem`
        # lands in src/control/flywheel/.
        owner = render.find_class(SRC_ROOT, args.requires)
        if owner and owner["definition"]:
            try:
                directory = owner["path"].parent.relative_to(
                    SRC_ROOT / "control"
                ).as_posix()
            except ValueError:
                directory = None  # subsystem lives outside src/control/

    if directory is None:
        # Fall back to the name with any trailing "subsystem"/"command" token
        # removed: `FlywheelSubsystem` -> `flywheel`.
        base = names_mod.tokenize(args.name)
        if base and base[-1] == kind:
            base = base[:-1]
        directory = "_".join(base)

    if not directory:
        raise ScaffoldError("could not derive a feature directory; pass --dir")

    target_dir = SRC_ROOT / "control" / directory
    # Join whatever namespace this directory already uses rather than adding a
    # second one beside it (src/control/chassis/ is `src::chassis`, not
    # `src::control::chassis`). New directories get the modern convention.
    namespace = args.namespace or render.dominant_namespace(target_dir)

    nm = names_mod.derive(
        args.name,
        kind,
        directory,
        args.class_override,
        args.instance_override,
        namespace,
    )

    existing = render.find_class(SRC_ROOT, nm["class_name"])
    if existing and existing["definition"] and not args.force:
        raise ScaffoldError(
            "class %s already exists at src/%s (pass --force to generate anyway)"
            % (nm["class_name"], existing["include"])
        )

    values = dict(nm)
    requires = None

    if kind == "command":
        requires = resolve_requires(args, target_dir, nm["namespace"])
        values.update(requires)

    files = []
    if args.header_only:
        files.append((target_dir / (nm["stem"] + ".hpp"),
                      render.render("%s_inline.hpp.tmpl" % kind, values)))
    else:
        files.append((target_dir / (nm["stem"] + ".hpp"),
                      render.render("%s.hpp.tmpl" % kind, values)))
        files.append((target_dir / (nm["stem"] + ".cpp"),
                      render.render("%s.cpp.tmpl" % kind, values)))

    return nm, requires, files


def main(argv):
    args = parse_args(argv)
    robots = resolve_robots(args.robots)
    nm, requires, files = build(args)

    written = render.write_files(files, dry_run=args.dry_run, force=args.force)
    fmt = (
        {"available": None, "clean": None, "checked": [], "note": "skipped (--dry-run)"}
        if args.dry_run
        else render.clang_format([p for p, _ in files], REPO_ROOT)
    )

    subsystem_instance = requires["requires_instance"] if requires else None
    plans = [wiring.plan(PROJECT_ROOT, r, nm, subsystem_instance) for r in robots]

    undo = "rm -r %s" % (files[0][0].parent.relative_to(REPO_ROOT))
    result = {
        "ok": True,
        "dry_run": args.dry_run,
        "kind": args.kind,
        "names": nm,
        "requires": requires,
        "files": written,
        "format": fmt,
        "wiring": plans,
        "undo": undo,
    }

    if args.as_json:
        print(json.dumps(result, indent=2, default=str))
    else:
        print_report(result, args.dry_run)
    return 0


def print_report(result, dry_run):
    out = sys.stdout.write
    verb = "Would create" if dry_run else "Created"
    out("\n%s:\n" % verb)
    for f in result["files"]:
        rel = Path(f["path"]).relative_to(REPO_ROOT)
        out("  %s%s\n" % (rel, "  (overwritten)" if f["existed"] else ""))

    fmt = result["format"]
    if fmt["note"]:
        out("  clang-format: %s\n" % fmt["note"])
    elif fmt["clean"]:
        out("  clang-format: clean\n")

    if result["requires"] and not result["requires"]["found"]:
        out("\n  note: `%s` was not found under src/; using the include you gave.\n"
            % result["requires"]["requires_class"])

    if len(result["wiring"]) > 1:
        out("\nWiring checklist for %d robots: %s\n"
            % (len(result["wiring"]), ", ".join(p["robot"] for p in result["wiring"])))
        out("  The generated files are shared by every robot, but each robot's\n"
            "  control file needs its own registration.\n")

    for plan in result["wiring"]:
        rel = Path(plan["control_file"]).relative_to(REPO_ROOT)
        out("\nTODO -- wire into %s\n" % rel)
        if plan["tag"]:
            out("  (registration functions on this robot use the tag `%s`)\n" % plan["tag"])
        for step in plan["steps"]:
            where = "line %s" % step["line"] if step["line"] else "location not found"
            mark = " [already present]" if step["already_present"] else ""
            out("\n  %d. %-12s %s%s\n" % (step["n"], where, step["where"], mark))
            for line in step["snippet"].splitlines():
                out("       %s\n" % line)
            if step["note"]:
                out("       ^ %s\n" % step["note"])
        for warning in plan["warnings"]:
            out("\n  ! %s\n" % warning)

    if not result["wiring"]:
        out("\n(no robots selected -- nothing to wire)\n")

    out("\nUndo: %s\n\n" % result["undo"])


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except (ScaffoldError, render.RenderError, names_mod.NameError_) as exc:
        sys.stderr.write("scaffold: error: %s\n" % exc)
        sys.exit(1)
