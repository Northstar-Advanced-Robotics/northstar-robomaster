#!/usr/bin/env python3
"""
Generate boilerplate for a new subsystem or command.

Usage (run from northstar-robomaster-project/):

    python3 build_tools/new_control.py subsystem FlywheelCooler
    python3 build_tools/new_control.py command SpinUp --subsystem FlywheelCooler

By default files are placed in src/control/<snake_case_name>/ (for subsystems)
or next to the subsystem they operate on (for commands). Use --dir to override,
e.g. --dir src/control/flywheel.

The generated files follow the conventions used across this codebase:
snake_case file names, src::control::<name> namespaces, and the standard
tap::control::Subsystem / tap::control::Command overrides. New .cpp files are
picked up automatically by the SCons build (it globs src/), so after generating
you can immediately build.

These are also available as VS Code tasks: "New Subsystem" and "New Command"
(Ctrl+Shift+P -> Tasks: Run Task).
"""

import argparse
import os
import re
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROL_DIR = os.path.join("src", "control")


def to_snake(name):
    """FlywheelCooler -> flywheel_cooler"""
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def to_camel(name):
    """flywheel_cooler -> FlywheelCooler (passes CamelCase through unchanged)"""
    return "".join(part.capitalize() for part in to_snake(name).split("_"))


def validate_class_name(name):
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        sys.exit(f"error: '{name}' is not a valid C++ class name")


SUBSYSTEM_HPP = """#ifndef {guard}
#define {guard}

#include "tap/control/subsystem.hpp"

namespace src::control::{namespace}
{{
/**
 * TODO: Describe what the {class_name}Subsystem controls.
 */
class {class_name}Subsystem : public tap::control::Subsystem
{{
public:
    {class_name}Subsystem(tap::Drivers* drivers);

    /** Called once when the subsystem is registered. Set up motors/IO here. */
    void initialize() override;

    /** Called repeatedly by the scheduler. Update outputs here. */
    void refresh() override;

    /** Called when the robot is in a safe-disconnect state. Stop actuators here. */
    void refreshSafeDisconnect() override {{}}

    const char* getName() const override {{ return "{display_name}"; }}

private:
}};

}}  // namespace src::control::{namespace}

#endif  // {guard}
"""

SUBSYSTEM_CPP = """#include "{header}"

#include "tap/drivers.hpp"

namespace src::control::{namespace}
{{
{class_name}Subsystem::{class_name}Subsystem(tap::Drivers* drivers)
    : tap::control::Subsystem(drivers)
{{
}}

void {class_name}Subsystem::initialize() {{}}

void {class_name}Subsystem::refresh() {{}}

}}  // namespace src::control::{namespace}
"""

COMMAND_HPP = """#ifndef {guard}
#define {guard}

#include "tap/control/command.hpp"

{subsystem_include}
namespace src::control::{namespace}
{{
/**
 * TODO: Describe what the {class_name}Command does.
 */
class {class_name}Command : public tap::control::Command
{{
public:
    {class_name}Command({subsystem_type}* subsystem);

    /** Called once each time the command is scheduled. */
    void initialize() override;

    /** Called repeatedly while the command is scheduled. */
    void execute() override;

    /** Called once when the command finishes or is interrupted. */
    void end(bool interrupted) override;

    /** @return true if the command may be scheduled right now. */
    bool isReady() override;

    /** @return true when the command is done and should be descheduled. */
    bool isFinished() const override;

    const char* getName() const override {{ return "{display_name}"; }}

private:
    {subsystem_type}* subsystem;
}};

}}  // namespace src::control::{namespace}

#endif  // {guard}
"""

COMMAND_CPP = """#include "{header}"

namespace src::control::{namespace}
{{
{class_name}Command::{class_name}Command({subsystem_type}* subsystem) : subsystem(subsystem)
{{
    addSubsystemRequirement(subsystem);
}}

void {class_name}Command::initialize() {{}}

void {class_name}Command::execute() {{}}

void {class_name}Command::end(bool) {{}}

bool {class_name}Command::isReady() {{ return true; }}

bool {class_name}Command::isFinished() const {{ return false; }}

}}  // namespace src::control::{namespace}
"""


def write_file(path, content):
    if os.path.exists(path):
        sys.exit(f"error: {path} already exists, refusing to overwrite")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(content)
    print(f"created {os.path.relpath(path, PROJECT_ROOT)}")


def find_subsystem_header(subsystem_snake):
    """Search src/control for an existing <subsystem>_subsystem.hpp."""
    target = f"{subsystem_snake}_subsystem.hpp"
    for root, _, files in os.walk(os.path.join(PROJECT_ROOT, "src")):
        if target in files:
            return os.path.relpath(os.path.join(root, target), os.path.join(PROJECT_ROOT, "src"))
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("kind", choices=["subsystem", "command"])
    parser.add_argument("name", help="Class name, e.g. FlywheelCooler (Subsystem/Command suffix is added)")
    parser.add_argument(
        "--subsystem",
        help="(command only) class name of the subsystem this command requires, e.g. FlywheelCooler",
    )
    parser.add_argument(
        "--dir",
        help="output directory relative to the project root (default: src/control/<name>)",
    )
    args = parser.parse_args()

    name = to_camel(args.name)
    validate_class_name(name)
    snake = to_snake(name)

    if args.kind == "subsystem":
        out_dir = args.dir or os.path.join(CONTROL_DIR, snake)
        namespace = os.path.basename(out_dir).replace("-", "_")
        hpp_name = f"{snake}_subsystem.hpp"
        cpp_name = f"{snake}_subsystem.cpp"
        fmt = dict(
            guard=f"{snake.upper()}_SUBSYSTEM_HPP_",
            namespace=namespace,
            class_name=name,
            display_name=f"{snake} subsystem",
            header=hpp_name,
        )
        write_file(os.path.join(PROJECT_ROOT, out_dir, hpp_name), SUBSYSTEM_HPP.format(**fmt))
        write_file(os.path.join(PROJECT_ROOT, out_dir, cpp_name), SUBSYSTEM_CPP.format(**fmt))
        print(
            f"\nNext steps:\n"
            f"  1. Add motors/IO to {name}Subsystem and fill in initialize()/refresh().\n"
            f"  2. Instantiate and register it in the relevant src/robot/<robot>/*_control.cpp."
        )
    else:
        if not args.subsystem:
            sys.exit("error: command generation requires --subsystem <SubsystemClassName>")
        sub_name = to_camel(args.subsystem)
        validate_class_name(sub_name)
        sub_snake = to_snake(sub_name)

        sub_header = find_subsystem_header(sub_snake)
        if sub_header is None:
            print(
                f"warning: could not find {sub_snake}_subsystem.hpp under src/; "
                f"you will need to fix the #include in the generated header"
            )
            sub_header = f"control/{sub_snake}/{sub_snake}_subsystem.hpp"

        out_dir = args.dir or os.path.dirname(os.path.join("src", sub_header))
        namespace = os.path.basename(out_dir).replace("-", "_")
        hpp_name = f"{snake}_command.hpp"
        cpp_name = f"{snake}_command.cpp"

        # Figure out the subsystem's namespace from its header so the generated
        # code can name it fully qualified if it lives elsewhere.
        sub_ns_match = None
        sub_header_abs = os.path.join(PROJECT_ROOT, "src", sub_header)
        if os.path.exists(sub_header_abs):
            with open(sub_header_abs) as f:
                sub_ns_match = re.search(r"namespace\s+([\w:]+)", f.read())
        sub_namespace = sub_ns_match.group(1) if sub_ns_match else f"src::control::{namespace}"
        subsystem_type = (
            f"{sub_name}Subsystem"
            if sub_namespace == f"src::control::{namespace}"
            else f"{sub_namespace}::{sub_name}Subsystem"
        )

        fmt = dict(
            guard=f"{snake.upper()}_COMMAND_HPP_",
            namespace=namespace,
            class_name=name,
            display_name=f"{snake} command",
            header=hpp_name,
            subsystem_include=f'#include "{sub_header}"\n',
            subsystem_type=subsystem_type,
        )
        write_file(os.path.join(PROJECT_ROOT, out_dir, hpp_name), COMMAND_HPP.format(**fmt))
        write_file(os.path.join(PROJECT_ROOT, out_dir, cpp_name), COMMAND_CPP.format(**fmt))
        print(
            f"\nNext steps:\n"
            f"  1. Fill in initialize()/execute()/end() in {name}Command.\n"
            f"  2. Instantiate it and map it to an input in the relevant "
            f"src/robot/<robot>/*_control.cpp."
        )


if __name__ == "__main__":
    main()
