NorthStarFleet2025
==================

Embedded control code for NorthStar's RoboMaster robots, running on a DJI type C
microcontroller board and built on the `taproot <https://gitlab.com/aruw/controls/taproot>`_
framework.

These pages are generated from the comments in ``northstar-robomaster-project/src``. If something
here is wrong, fix the comment in the source rather than the page.

The robots
----------

One codebase builds every robot. The build target selects which ``robot/<target>/`` directory is
compiled in, and each supplies its own constants and its own ``initSubsystemCommands``:

==============  ==========================================================================
Target          What it is
==============  ==========================================================================
``standard``    The general-purpose infantry robot: chassis, turret, 17mm launcher.
``hero``        Fires a single large 42mm projectile; has a kicker feeding the flywheels.
``sentry``      Runs autonomously with no operator. See ``StateMachineSubsystem``.
``turret``      Firmware for a second board mounted on the turret itself.
``testbed``     A bench fixture. Which hardware is compiled in is chosen in ``test_def.hpp``.
==============  ==========================================================================

How the code is organised
-------------------------

taproot's scheduler is built from three pieces, and nearly everything here is one of them:

**Subsystems** own hardware. Exactly one command may use a subsystem at a time, which is what stops
two pieces of code fighting over the same motor. ``refresh()`` runs every control loop iteration
(500 Hz) whether or not a command is scheduled.

**Commands** do things with subsystems. A command declares which subsystems it needs, then runs
``initialize`` / ``execute`` / ``isFinished`` / ``end``. Scheduling a command that needs a busy
subsystem interrupts whatever held it.

**Governors** gate commands. A governor answers "may this run right now?" -- used for firing
conditions like heat limits, flywheel readiness, and whether auto-aim is on target.

Input reaches commands through ``ControlOperatorInterface``, never from the remote directly.

Coordinate frames
-----------------

``ChassisSubsystem`` and ``ChassisOdometry`` share one right-handed frame: **+X forward, +Y left**,
headings counterclockwise positive. Odometry output can be handed to the chassis unchanged --
``StateMachineSubsystem`` calls ``setVelocityFieldDrive(vel.x, vel.y, rot)`` with no axis swap.

One exception is the easiest way to introduce a sign bug: the ``rotational`` argument of the chassis
drive methods is **clockwise** positive, even though headings, ``getChassisYaw()`` and the measured
``getChassisRotationSpeed()`` are counterclockwise positive. Negate a counterclockwise quantity
before passing it in as ``rotational``.

The HUD's ``Projections`` space is separate again (+X right, +Y forward, +Z up) and must be
converted into.

Angles are radians unless a comment says otherwise, and wheel speeds are motor-shaft RPM, not wheel
RPM.

Where to start reading
----------------------

- ``src/robot/<target>/<target>_control.cpp`` -- wires up one robot; the best map of what exists.
- ``src/control/chassis/chassis_subsystem.hpp`` -- driving, power limiting, and the frame convention.
- ``src/control/turret/`` -- aiming. The controllers in ``algorithms/`` are the interesting part.
- ``src/communication/serial/vision_comms.hpp`` -- the link to the vision computer.

A caution
---------

Some classes here are not built into any robot -- the older ``clientDisplay/indicators`` HUD and
the turret-MCB CAN path in particular. These carry a ``@deprecated`` or ``@warning`` note saying so.
Check for one before building on a class.

.. toctree::
    :maxdepth: 2
    :caption: By subsystem:

    subsystems/chassis
    subsystems/turret
    subsystems/agitator
    subsystems/flywheel
    subsystems/hopper_kicker
    subsystems/governors
    subsystems/client_display
    subsystems/communication
    subsystems/robots
    subsystems/util

.. toctree::
    :maxdepth: 2
    :caption: Full reference:

    api/library_root

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
