"""Read-only inspection of the per-robot `*_control.cpp` files.

This module never writes. It opens each robot's control file only to resolve
facts that would otherwise have to be guessed, and turns them into a precise,
copy-pasteable checklist:

  - the registration function TAG, which is NOT the robot name: it is
    Standard / Hero / Sentry, but `Test` for testbed and `Soldier` for turret
  - the line number of each of the six wiring sites
  - whether a `// <dir>` include banner already exists
  - whether the trailing `/* ... */` trigger list exists (turret and testbed
    have none)

The anchor regexes below were verified against all five control files.
"""

import re

ROBOTS = ("standard", "hero", "sentry", "turret", "testbed")

_RE_USING = re.compile(r"^using\s")
_RE_DRIVERS = re.compile(r"^driversFunc\s+drivers\s*=")
_RE_NS_OPEN = re.compile(r"^namespace\s+(\w+_control)\s*$")
_RE_INIT = re.compile(r"^void\s+initializeSubsystems\s*\(")
_RE_REGISTER = re.compile(r"^void\s+register(\w+)Subsystems\s*\(")


class Anchors:
    def __init__(self, robot, path, lines):
        self.robot = robot
        self.path = path
        self.lines = lines
        self.text = "".join(lines)
        self.warnings = []

        self.tag = None
        self.register_line = None
        self.init_line = None
        self.default_line = None
        self.iomap_line = None
        self.using_line = None
        self.drivers_line = None
        self.namespace = None
        self.namespace_line = None

        self._scan()

    def _find(self, regex, label, group=None):
        for i, line in enumerate(self.lines, start=1):
            m = regex.match(line)
            if m:
                return (i, m.group(group)) if group else (i, None)
        self.warnings.append("could not locate %s" % label)
        return (None, None)

    def _scan(self):
        self.register_line, self.tag = self._find(
            _RE_REGISTER, "register<Tag>Subsystems()", group=1
        )
        self.init_line, _ = self._find(_RE_INIT, "initializeSubsystems()")
        self.using_line, _ = self._find(_RE_USING, "the `using namespace` block")
        self.drivers_line, _ = self._find(_RE_DRIVERS, "the `driversFunc drivers` line")
        self.namespace_line, self.namespace = self._find(
            _RE_NS_OPEN, "the `namespace <robot>_control` block", group=1
        )

        if self.tag:
            self.default_line, _ = self._find(
                re.compile(r"^void\s+setDefault%sCommands\s*\(" % re.escape(self.tag)),
                "setDefault%sCommands()" % self.tag,
            )
            self.iomap_line, _ = self._find(
                re.compile(r"^void\s+register%sIoMappings\s*\(" % re.escape(self.tag)),
                "register%sIoMappings()" % self.tag,
            )

    def banner_line(self, directory):
        """Line number of an existing `// <dir>` include banner, or None."""
        want = re.compile(r"^//\s*%s\s*$" % re.escape(directory), re.I)
        for i, line in enumerate(self.lines, start=1):
            if want.match(line.rstrip()):
                return i
        return None

    def contains(self, needle):
        return needle in self.text


def control_file(project_root, robot):
    return project_root / "src" / "robot" / robot / ("%s_control.cpp" % robot)


def _step(n, line, where, snippet, done=False, note=None):
    return {
        "n": n,
        "line": line,
        "where": where,
        "snippet": snippet,
        "already_present": done,
        "note": note,
    }


def plan(project_root, robot, names, subsystem_instance=None):
    """Build the wiring checklist for one robot. Read-only."""
    path = control_file(project_root, robot)
    if not path.is_file():
        return {
            "robot": robot,
            "control_file": str(path),
            "tag": None,
            "steps": [],
            "warnings": ["no control file at %s" % path],
        }

    a = Anchors(robot, path, path.read_text(errors="replace").splitlines(keepends=True))
    tag = a.tag or "<Tag>"
    directory = names["directory"]
    cls = names["class_name"]
    inst = names["instance"]
    include = '#include "%s"' % names["include"]

    # Without a `using namespace` block the declaration has to be qualified.
    decl_type = cls if a.using_line else "%s::%s" % (names["namespace"], cls)

    banner = a.banner_line(directory)
    include_anchor = banner or a.using_line or a.namespace_line
    steps = [
        _step(
            1,
            include_anchor,
            (
                'add to the existing "// %s" include banner' % directory
                if banner
                else "start a new include banner in the include block"
            ),
            include if banner else "// %s\n%s" % (directory, include),
            done=a.contains(include),
        )
    ]

    if a.using_line:
        steps.append(
            _step(
                2,
                a.drivers_line or a.using_line,
                "add to the `using namespace` block",
                "using namespace %s;" % names["namespace"],
                done=a.contains("using namespace %s;" % names["namespace"]),
            )
        )
    else:
        # testbed has no `using namespace` block at all.
        steps.append(
            _step(
                2,
                a.namespace_line,
                "this file has no `using namespace` block -- "
                "write the type fully qualified instead",
                "%s::%s" % (names["namespace"], cls),
                note="no `using namespace` line to add to; nothing to paste here",
            )
        )

    if names["kind"] == "subsystem":
        steps += [
            _step(
                3,
                a.namespace_line,
                "declare at file scope inside `namespace %s`" % (a.namespace or "<robot>_control"),
                "%s %s(drivers());" % (decl_type, inst),
                done=re.search(r"\b%s\s*\(" % re.escape(inst), a.text) is not None,
            ),
            _step(
                4,
                a.init_line,
                "inside initializeSubsystems()",
                "%s.initialize();" % inst,
                done=a.contains("%s.initialize();" % inst),
            ),
            _step(
                5,
                a.register_line,
                "inside register%sSubsystems()" % tag,
                "drivers->commandScheduler.registerSubsystem(&%s);" % inst,
                done=a.contains("registerSubsystem(&%s);" % inst),
            ),
            _step(
                6,
                a.default_line,
                "optional -- inside setDefault%sCommands()" % tag,
                "%s.setDefaultCommand(&yourCommand);" % inst,
                note="only if this subsystem should run something when idle",
            ),
        ]
    else:
        target = subsystem_instance or "yourSubsystem"
        steps += [
            _step(
                3,
                a.namespace_line,
                "declare at file scope inside `namespace %s`" % (a.namespace or "<robot>_control"),
                "%s %s(&%s);" % (decl_type, inst, target),
                done=re.search(r"\b%s\s*\(" % re.escape(inst), a.text) is not None,
                note="`%s` must already be declared above this line" % target,
            ),
            _step(
                4,
                a.iomap_line,
                "bind it to an input -- declare the Trigger at file scope, "
                "then list its name in the trailing /* */ block of "
                "register%sIoMappings()" % tag,
                "Trigger xPressed%s =\n"
                "    TriggerHelpers::button(drivers(), Remote::Key::X).toggleOnTrue(&%s);"
                % (cls, inst),
                note="Triggers self-register; the /* */ list is for readability only",
            ),
            _step(
                5,
                a.default_line,
                "or -- make it the subsystem's idle behaviour instead of a Trigger",
                "%s.setDefaultCommand(&%s);" % (target, inst),
                note="pick a Trigger or a default command, not usually both",
            ),
        ]

    if names["kind"] == "command" and a.iomap_line and "*/" not in a.text[
        a.text.find("register%sIoMappings" % tag) :
    ]:
        a.warnings.append(
            "register%sIoMappings() has no trailing /* */ trigger list; "
            "add the Trigger declaration at file scope anyway" % tag
        )

    if not a.using_line:
        a.warnings = [
            w
            for w in a.warnings
            if "`using namespace` block" not in w and "`driversFunc drivers`" not in w
        ]

    if robot == "testbed":
        a.warnings.append(
            "testbed gates its subsystems behind `#define USING_<FEATURE>` in "
            "src/robot/testbed/test_def.hpp, with the objects declared in a "
            "using_<feature>.hpp fragment. Follow that pattern rather than "
            "pasting directly into testbed_control.cpp."
        )

    return {
        "robot": robot,
        "control_file": str(path),
        "tag": a.tag,
        "namespace": a.namespace,
        "steps": steps,
        "warnings": a.warnings,
    }
