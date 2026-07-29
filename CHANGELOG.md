# Changelog

All notable changes to this project are documented in this file.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

## [Unreleased] — branch `docker_compatibility`, 2026-07-29

Addresses GitHub issues #71 (build warnings), #67 (make compiling better),
#72 (subsystem/command template), #65 (unit test compatibility), and parts of
#70/#73/#74 (code cleanup, documentation, unused code).

### Fixed — functional bugs

- **TURRET robot target could not compile at all.** Roughly a dozen files used
  `#elif TURRET` / `#ifdef TURRET`, but the build defines `TARGET_TURRET`, so no
  branch of the robot conditionals matched and `drivers_singleton.hpp` produced a
  syntax error. All robot-type conditionals now use the `#elif defined(TARGET_X)`
  form (`main.cpp`, `drivers_singleton.{hpp,cpp}`, `robot_control.hpp`,
  `turret_control.cpp`, and the agitator/chassis/flywheel/kicker/turret/uart
  constants headers). `main.cpp` also guards `encoder`/`visionComms` usage, which
  the minimal turret MCB's `Drivers` does not have.
- **Hero chassis power look-up table typo** (`hero_chassis_constants.hpp`): the
  120 W entry was written `(120, 3'825)` — a comma expression, not a pair — so the
  table actually contained `{3825, 0}`. This corrupted power-to-max-wheel-speed
  interpolation on hero at high power limits. Now `{120, 3'825}`.
- **Client display string buffer over-read** (`atomic_graphics_objects.hpp`):
  `StringGraphic::setString` used `strncpy(string, newString, STRING_SIZE)`,
  leaving the buffer without a null terminator for strings of 31+ characters
  (several IMU-calibration HUD strings qualify). Later `stringLen`/`strncmp`
  calls could read past the buffer. Now truncates to `STRING_SIZE - 1` and always
  null-terminates.
- **Every robot's sources were compiled and linked into every build.** The
  per-robot filter in `src/SConscript` compared paths against `src/robot/...`,
  but source paths are relative to `src/` itself, so the filter never matched
  (e.g. sentry files were built into the STANDARD firmware). Paths fixed; each
  build now only compiles shared code plus its own robot directory. The same
  path bug also made the test-environment exclusion of `*_control.cpp` a no-op.
- **`scons build-tests` / `run-tests` / `build-sim` silently did nothing:** the
  tests/sim alias registration in `SConstruct` was nested inside the hardware
  branch and was unreachable. Restructured into a proper if/elif/else.
- **`robot=TARGET_STANDARD` (the documented form) was rejected** by the robot
  type matcher and fell through to an interactive prompt, which crashes with an
  `EOFError` in CI/scripted builds. Both `robot=STANDARD` and
  `robot=TARGET_STANDARD` now work; non-interactive builds fail with a clear
  message instead of a traceback.
- `chassis_subsystem.hpp` declared `getTurretYaw()` `inline` but defined it in
  the `.cpp`, leaving every other translation unit without a definition (22
  warnings, and a latent link hazard). The definition now lives in the header.

### Removed — dead multi-turret support

All robots have exactly one turret (every robot defined `NUM_TURRETS = 1`), so
the vestigial multi-turret plumbing was removed:

- `VisionComms`: `lastAimData`/`aimDataUpdated` are now single values instead of
  one-element arrays; `getLastAimData()`, `isAimDataUpdated()`, and
  `getSomeTurretHasTarget()` no longer take a `turretID`; the aim-data decode
  loop was flattened.
- Removed the `turretID` constructor parameter and member from
  `TurretCVControlCommand`, `TurretUserControlCommand`,
  `TurretUserWorldRelativeCommand`, and `CvOnTargetGovernor`;
  `isAimingWithinLaunchingTolerance()` (and its interface in
  `turret_cv_control_command_template.hpp`) no longer takes a turret ID.
- Removed the `NUM_TURRETS` constant from all four robot turret constants
  headers.
- This also fixed two latent bugs: `sentry_cv_manager_command.cpp` indexed the
  1-element aim-data arrays with `isAimDataUpdated(1)` (out-of-bounds read),
  and `CvOnTargetGovernor` read an uninitialized `turretID` member (the
  constructor discarded its parameter).

### Added

- **Subsystem/command generator** (issue #72): `build_tools/new_control.py`
  scaffolds a new subsystem or command following project conventions
  (snake_case files, `src::control::<name>` namespace, standard taproot
  overrides, doxygen comment stubs). Available as VS Code tasks
  **New Subsystem** and **New Command** (with input prompts) and documented in
  the README.
- **Project-level gmock mocks** required by the unit-test environment but never
  written: `src/mock/control_operator_interface_mock.hpp` and
  `src/mock/turret_motor_mock.hpp`.
- `CHANGELOG.md` (this file).

### Running the unit tests and the simulator

Both the unit tests and the simulator now build and run. All commands are run
from `northstar-robomaster-project/` (same as hardware builds).

**Unit tests** — compile the robot code for your own computer (not the ARM
board) and run it against googletest:

```
pipenv run scons run-tests robot=STANDARD          # build + run all tests
pipenv run scons build-tests robot=STANDARD        # build only
pipenv run scons run-tests-gcov robot=STANDARD     # run + code coverage report
```

Or use the VS Code tasks **Run Tests - Debug** / **Build Tests - Debug**.

- Test files live in `test/` (see `test/my_first_test.cpp` for a minimal
  example). Any `*.cpp` there is picked up automatically; write tests with
  `TEST(...)` / `TEST_F(...)` and gmock.
- In the test build, `PLATFORM_HOSTED` and `ENV_UNIT_TESTS` are defined:
  hardware drivers are replaced by gmock mocks (`taproot/src/tap/mock/` plus
  the project mocks in `src/mock/`), and `main.cpp` and
  `src/robot/**/*_control.cpp` are excluded.
- Requirements (already set up in the devcontainer): `libgtest-dev`/
  `libgmock-dev` system packages and the `taproot-scripts` submodule
  (`git submodule update --init taproot-scripts`).

**Simulator** — build the full robot program (including `main.cpp` and the
robot control files) as a native executable with simulated DJI motors:

```
pipenv run scons build-sim robot=STANDARD          # build only
pipenv run scons run-sim robot=STANDARD            # build + launch
```

Or use the VS Code tasks **Build Sim - Debug** / **Run Sim - Debug**. The
executable lands at `build/sim/scons-<profile>/TARGET_<ROBOT>/northstar-robomaster-project.elf`
and runs the real robot main loop; motor CAN traffic goes through taproot's
`DjiMotorSimHandler` motor simulation.

Sim limitations to be aware of: in the sim, `PLATFORM_HOSTED` is defined but
`ENV_UNIT_TESTS` is not, so the code runs against no-op stubs rather than
mocks — the PWM encoder reports offline, the hardware random number generator
and IMU do nothing, and there is no remote/ref-serial input, so the startup
remote-wait loop in `main.cpp` runs its full timeout before the scheduler
starts. It is a "does the whole program run" harness, not a physics
simulation.

### Unit tests (issue #65)

`scons run-tests` now builds, links, and passes (the `my_first_test` example).
Work required beyond the SConstruct and mock fixes above:

- Initialized the `taproot-scripts` git submodule (provides the `run_gcov`
  SCons tool used by the tests target).
- Installed `libgtest-dev`/`libgmock-dev` in the container and added them to
  `.devcontainer/Dockerfile` so future containers have them.
- Guarded hardware-only code for the hosted environment: the PWM encoder
  (STM32 Timer12), the hardware `RandomNumberGenerator` in
  `chassis_beyblade_command.cpp`, the `TurretMotorDJI` downcast in the two
  world-frame turret IMU controllers, and the robot `Drivers` constructors
  (which initialized members that don't exist in the unit-test build).
- Removed a broken `#include ""` line from the generated
  `taproot/src/tap/drivers.hpp` (lbuild artifact). **Note:** this file is
  generated; if taproot is ever regenerated the fix must be re-applied or the
  empty option in the lbuild config fixed upstream.

### Simulator (`build-sim` / `run-sim`) fixed

The sim target (unreachable before the SConstruct alias fix) now builds,
links, and runs. Additional fixes it needed:

- Added missing `#include <optional>` to the standard/sentry/testbed flywheel
  constants headers and `multi_shot_cv_command_mapping.hpp` (the ARM
  toolchain provided it transitively; host GCC does not).
- `main.cpp`'s hosted branch used an old taproot sim API
  (`tap::motorsim::SimHandler`, `tap::communication::TCPServer`) that this
  project's generated taproot doesn't contain; ported to the current
  `tap::motor::motorsim::DjiMotorSimHandler` and dropped the TCP-server wait
  (that module isn't generated).
- `PwmEncoder` now has no-op hosted stubs (constructor, `initialize`,
  `update`, `isOnline` → `false`) instead of simply not being compiled, since
  the sim links the real robot `Drivers` which owns one.

### Changed — warning cleanup (issue #71)

All five robot targets (STANDARD, HERO, SENTRY, TURRET, TEST_BED) now build
with **zero compiler warnings** (previously up to ~200 per target):

- Reordered constructor member-initializer lists to match declaration order
  (`-Wreorder`) in the chassis, hopper, state machine, flywheel, vision comms,
  and sentry CV manager classes.
- Removed or unnamed unused parameters and variables (`-Wunused-parameter`,
  `-Wunused-variable`, `-Wunused-but-set-variable`) across chassis, turret,
  flywheel, kicker, hopper, and robot control files; used `[[maybe_unused]]`
  where the value is only consumed by conditionally compiled code.
- `CubicBezier::CurveData`: removed the `modm_packed` attribute that GCC was
  ignoring (non-POD members) and pinned the wire-format layout with a
  `static_assert` instead.
- `test_subsystem.hpp`: `getName()` now `const override` instead of hiding the
  virtual base method (`-Woverloaded-virtual`).
- Explicit `uint8_t` cast for the flywheel interpolator size (`-Wnarrowing`);
  explicit `Ramp(0.0f)` construction for the chassis ramp array; added the
  missing `.velocityPIDFeedForwardGain` initializer to hero's kicker config;
  removed a stray line-continuation backslash in a sentry comment
  (`-Wcomment`).

### Changed — build tooling and docs

- Removed the phantom `DRONE`/`ENGINEER` robot types from the valid target list
  (no `src/robot/` directories or conditional branches exist for them; selecting
  them could never build) and updated the `scons` usage text to list the real
  targets: STANDARD, HERO, SENTRY, TURRET, TEST_BED.
- README: fixed the stale "Selecting and using robot types" section (the
  referenced `robot-type/robot_type.hpp` no longer exists — robot selection is
  via `robot=` or the interactive prompt, IntelliSense via the per-robot C/C++
  configuration), fixed the clone URL to point at this repository, and added a
  "Creating a new subsystem or command" section for the generator.

### Verified

- All five hardware targets build cleanly (exit 0, zero warnings).
- Switching robot types recompiles **zero** files (objects are cached per robot
  under `build/hardware/scons-<profile>/TARGET_<ROBOT>/`); only a relink is
  performed (issue #67).
- `scons run-tests robot=STANDARD` passes.
- `scons build-sim robot=STANDARD` builds and links; the resulting executable
  launches and runs the robot main loop without crashing.

### Deliberately not changed

Coordinate-frame refactors (#58/#60/#61/#62), the Triggers-everywhere refactor
(#66), and hardware-dependent tuning work (#41, #39, #31, #12) change robot
behavior and need validation on physical robots. Candidate unused files
(`src/my_first_file.hpp` + `test/my_first_test.cpp` — the only working test
example — and `src/control/testSubsystem/`, used by the testbed) were kept
pending a team decision.
