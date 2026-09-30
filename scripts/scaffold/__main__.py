#!/usr/bin/env python3
"""Scaffold a new Taproot subsystem or command.

Creates the .hpp/.cpp under northstar-robomaster-project/src/control/<dir>/
and formats them with clang-format. It never modifies an existing file.

    python3 scripts/scaffold subsystem Flywheel
    python3 scripts/scaffold command SpinUp --requires FlywheelSubsystem
"""

import argparse
import json
import re
import sys
from pathlib import Path

import names as names_mod
import render

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
    p.add_argument("--class", dest="class_override", default=None,
                   help="override the derived class name (e.g. for acronyms)")
    p.add_argument("--instance", dest="instance_override", default=None,
                   help="override the derived instance name")
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
        help="the subsystem this command requires, by file name: chassis_subsystem or chassis",
    )
    cmd.add_argument(
        "--requires-include",
        default=None,
        help="override the include path for --requires (default: found by searching src/)",
    )
    return parser.parse_args(argv)


_CLASS_DEF = re.compile(r"^[ \t]*class\s+(\w+)\s*(?::|\{)", re.M)


def _subsystem_stems(directory):
    """File stems of the subsystem headers under `directory`, e.g. chassis_subsystem."""
    return sorted(p.stem for p in Path(directory).rglob("*_subsystem.hpp"))


def find_subsystem(value, directory=None):
    """Find the subsystem named by `value` and describe it like render.find_class.

    `value` is normally the header's file name -- `chassis_subsystem`,
    `chassis_subsystem.hpp` or just `chassis` -- which is unambiguous because no two
    subsystem headers share a name. An exact class name (`PoopSubsystem`) also
    works, for acronyms or the odd subsystem whose file is named differently.
    """
    if value.endswith(".hpp"):
        value = value[: -len(".hpp")]
    tokens = names_mod.tokenize(value)
    if tokens and tokens[-1] == "subsystem":
        tokens = tokens[:-1]
    if tokens:
        stem = "_".join(tokens + ["subsystem"])
        headers = sorted(SRC_ROOT.rglob(stem + ".hpp"))
        if len(headers) == 1:
            match = _CLASS_DEF.search(headers[0].read_text(errors="replace"))
            if match:
                found = render.find_class(SRC_ROOT, match.group(1))
                return dict(found, class_name=match.group(1))

    found = render.find_class(SRC_ROOT, value)
    if found and found["definition"]:
        return dict(found, class_name=value)

    where = SRC_ROOT / "control" / directory if directory else None
    if where is None or not _subsystem_stems(where):
        where = SRC_ROOT / "control"
    raise ScaffoldError(
        'no subsystem "%s". Subsystems in src/%s/: %s'
        % (value, where.relative_to(SRC_ROOT).as_posix(),
           ", ".join(_subsystem_stems(where)) or "(none)")
    )


def resolve_requires(args, subsystem, target_dir, command_namespace):
    """Work out how the command should refer to the subsystem it requires."""
    found = subsystem
    cls = found["class_name"]

    include = args.requires_include or found["include"]

    # Same directory -> plain filename, matching play_song_command.hpp.
    if found["path"].parent == target_dir:
        include = found["path"].name

    qualified = cls
    if found["namespace"] and found["namespace"] != command_namespace:
        qualified = "%s::%s" % (found["namespace"], cls)

    tokens = names_mod.tokenize(cls)
    if len(tokens) > 1 and tokens[-1] == "subsystem":
        tokens = tokens[:-1]
    instance = names_mod.lower_camel(tokens)

    return {
        "requires_class": qualified,
        "requires_include": include,
        "requires_instance": instance,
    }


def build(args):
    kind = args.kind

    directory = args.directory.strip("/") if args.directory else None

    subsystem = None
    if kind == "command":
        subsystem = find_subsystem(args.requires, directory)

    if directory is None and subsystem:
        # A command belongs beside the subsystem it drives, not in a directory
        # named after itself: `command SpinUp --requires flywheel` lands in
        # src/control/flywheel/.
        try:
            directory = subsystem["path"].parent.relative_to(
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
        requires = resolve_requires(args, subsystem, target_dir, nm["namespace"])
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
    nm, requires, files = build(args)

    written = render.write_files(files, dry_run=args.dry_run, force=args.force)
    fmt = (
        {"available": None, "clean": None, "checked": [], "note": "skipped (--dry-run)"}
        if args.dry_run
        else render.clang_format([p for p, _ in files], REPO_ROOT)
    )

    result = {
        "ok": True,
        "dry_run": args.dry_run,
        "kind": args.kind,
        "names": nm,
        "requires": requires,
        "files": written,
        "format": fmt,
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

    out("\n")


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except (ScaffoldError, render.RenderError, names_mod.NameError_) as exc:
        sys.stderr.write("scaffold: error: %s\n" % exc)
        sys.exit(1)
