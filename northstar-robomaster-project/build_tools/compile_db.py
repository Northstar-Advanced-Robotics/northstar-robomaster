# Generates compile_commands.json for editor tooling (clangd, VS Code C/C++, clang-tidy).
#
# SCons 3.x has no built-in compilation database support, so we re-run the whole SCons
# invocation under Bear (https://github.com/rizsotto/Bear), which records every compiler
# command that gets executed.
#
# Bear only sees commands that actually run, so an incremental build would otherwise produce a
# nearly empty database. To avoid that, every run is merged into a master database
# (build/compile_commands.all.json) keyed by object file. The project-level
# compile_commands.json is then regenerated from the master database, filtered down to the
# configuration that was just built (target env + profile + robot), so each source file
# appears exactly once with the flags of the most recent build.
#
# Pass "compiledb=false" on the command line to skip all of this.

import json
import os
import shutil
import subprocess
import sys
from os.path import abspath, isfile, join

from SCons.Script import ARGUMENTS, GetLaunchDir, GetOption

_INNER_RUN_ENV_VAR = "NORTHSTAR_COMPILEDB_INNER_RUN"
_ROBOT_DIR_PREFIX = "TARGET_"

MASTER_DB_NAME = "compile_commands.all.json"
RUN_DB_NAME = ".compile_commands.run.json"
OUTPUT_DB_NAME = "compile_commands.json"


def _should_wrap():
    if os.environ.get(_INNER_RUN_ENV_VAR):
        return False
    if ARGUMENTS.get("compiledb", "true").lower() in ("false", "0", "no", "off"):
        return False
    # Nothing gets compiled for these, so don't touch the database.
    if GetOption("clean") or GetOption("no_exec") or GetOption("help"):
        return False
    return True


def run_under_bear(project_dir, build_root, config_dir, robot_dir):
    """
    If this is the top-level SCons invocation, re-run it under Bear, update the compilation
    databases, and exit with SCons's exit code. Returns without doing anything when already
    running under Bear (or when disabled), in which case the build proceeds normally.

    project_dir: directory that receives compile_commands.json
    build_root:  top-level build directory (holds the master database)
    config_dir:  build directory for this env/profile, e.g. build/hardware/scons-release
    robot_dir:   name of the per-robot subdirectory of config_dir, e.g. TARGET_STANDARD
    """
    if not _should_wrap():
        return

    bear = shutil.which("bear")
    if bear is None:
        print(
            "compile_commands.json: 'bear' not found on PATH, skipping. "
            "Install it (sudo apt install bear) or pass compiledb=false to silence this."
        )
        return

    os.makedirs(build_root, exist_ok=True)
    run_db = join(build_root, RUN_DB_NAME)
    if isfile(run_db):
        os.remove(run_db)

    cmd = [bear, "--output", run_db, "--", sys.executable] + sys.argv
    env = dict(os.environ)
    env[_INNER_RUN_ENV_VAR] = "1"

    # Run from where the user launched scons so relative -C/-f arguments still resolve.
    proc = subprocess.Popen(cmd, cwd=GetLaunchDir(), env=env)
    try:
        returncode = proc.wait()
    except KeyboardInterrupt:
        # The child gets the same SIGINT; let it (and Bear) shut down cleanly.
        returncode = proc.wait()

    try:
        _update_databases(run_db, project_dir, build_root, config_dir, robot_dir)
    except Exception as e:  # Never fail the build over editor tooling.
        print("compile_commands.json: failed to update ({})".format(e))

    sys.exit(returncode)


def _entry_output(entry):
    output = entry.get("output")
    if not output:
        return None
    return abspath(join(entry.get("directory", ""), output))


def _entry_file(entry):
    return abspath(join(entry.get("directory", ""), entry["file"]))


def _load(path):
    if not isfile(path):
        return []
    with open(path) as f:
        return json.load(f)


def _write(path, entries):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(entries, f, indent=2)
    os.replace(tmp, path)


def _update_databases(run_db, project_dir, build_root, config_dir, robot_dir):
    new_entries = _load(run_db)
    if isfile(run_db):
        os.remove(run_db)

    # Merge this run into the master database. One object file is produced by exactly one
    # command, so the object path is the merge key; the newest command wins.
    master_path = join(build_root, MASTER_DB_NAME)
    merged = {}
    for entry in _load(master_path) + new_entries:
        key = _entry_output(entry) or _entry_file(entry)
        merged[key] = entry
    master = list(merged.values())
    _write(master_path, master)

    # Select the entries belonging to the configuration that was just built: everything under
    # config_dir except the object directories of other robots. The shared libraries (taproot,
    # modm) live next to the robot directories and are included for every robot.
    config_prefix = abspath(config_dir) + os.sep
    selected = {}
    for entry in master:
        output = _entry_output(entry)
        if output is None or not output.startswith(config_prefix):
            continue
        first_dir = output[len(config_prefix) :].split(os.sep, 1)[0]
        if first_dir.startswith(_ROBOT_DIR_PREFIX) and first_dir != robot_dir:
            continue
        selected[_entry_file(entry)] = entry

    entries = sorted(selected.values(), key=_entry_file)
    _write(join(project_dir, OUTPUT_DB_NAME), entries)
    print(
        "compile_commands.json: {} entries for {}/{}".format(
            len(entries), os.path.relpath(config_dir, build_root), robot_dir
        )
    )
