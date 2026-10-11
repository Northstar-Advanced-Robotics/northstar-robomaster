# Engineer Arm Architecture

_NorthStarFleet controls · ARC Season 2 engineer robot · last updated 2026-10-11_

This is the guide to how the engineer's arm code is organised: what each piece does, how the pieces talk
to each other, and the order to build them in. Read it top to bottom once; after that, use it as a reference.

> **Status.** This is the plan, not finished code. The **motors are not chosen** — the hardware in §9 is a
> budget-driven draft from mechanical and can change. The arm code is written so that it doesn't care.

---

## Contents

1. [The big idea](#1-the-big-idea)
2. [The three layers](#2-the-three-layers)
3. [Where the files go](#3-where-the-files-go)
4. [ArmSubsystem vs. ArmController](#4-armsubsystem-vs-armcontroller)
5. [What happens, step by step](#5-what-happens-step-by-step)
6. [Joints and actuators](#6-joints-and-actuators)
7. [The control loop](#7-the-control-loop)
8. [Gravity compensation](#8-gravity-compensation)
9. [Motor backends](#9-motor-backends)
10. [Build order](#10-build-order)
11. [Checklist: adding or swapping a motor](#11-checklist-adding-or-swapping-a-motor)
12. [Glossary](#12-glossary)
13. [Open questions](#13-open-questions)

---

## 1. The big idea

**The arm is built from a list of joints.**

You describe each joint once — where it sits, which way it turns, how heavy the part after it is, its limits,
and which motor drives it — put the joints in an array, and hand the array to the `ArmController`. The
controller works out everything else from that list: where the gripper is, how much torque each joint needs to
hold the arm up, and how to move safely.

Three rules follow from that:

| Rule | Why it matters |
|---|---|
| **Commands only set targets.** | A command says *"go here"* or *"hold"*. It never touches a motor. All motor output goes through one place, so the safety limits can't be skipped. |
| **The arm doesn't know what motors it has.** | Motors aren't chosen yet, and they may change. Swapping a motor should be a one-line change, not a rewrite. |
| **The arm's brain is plain C++.** | The same code runs on the robot, in the simulator, and in unit tests with fake motors. |

---

## 2. The three layers

```
 ┌────────────────────────────────────────────────────────────────────────────┐
 │  COMMANDS        hold · home · joint jog · cartesian jog · preset · follow │
 │                  "what the driver wants"  →  set a target, nothing else     │
 └───────────────────────────────┬────────────────────────────────────────────┘
                                 │ setJointTarget(), holdCurrent(), ...
 ┌───────────────────────────────▼────────────────────────────────────────────┐
 │  ArmSubsystem    Taproot's view of the arm. Each tick: compute dt,          │
 │                  call controller.update(dt). That's it.                    │
 └───────────────────────────────┬────────────────────────────────────────────┘
                                 │ update(dt)
 ┌───────────────────────────────▼────────────────────────────────────────────┐
 │  ArmController   THE BRAIN. Owns the joints. Each tick:                     │
 │                  read joints → limits → plan motion → PD + gravity → torque │
 └───────────────────────────────┬────────────────────────────────────────────┘
                                 │ commandTorque(N·m)          ▲ position(), velocity()
 ┌───────────────────────────────▼─────────────────────────────┴──────────────┐
 │  JointActuator × 6   one per joint. Turns "N·m at the joint" into whatever  │
 │                      the motor wants (DJI current, Damiao MIT frame, ...)   │
 └────────────────────────────────────────────────────────────────────────────┘
```

> **Naming note.** "Arm controller" means the `ArmController` software object. The **custom controller** is the
> physical leader arm the operator holds — it's just one input, used by `ArmCustomControllerCommand`.

---

## 3. Where the files go

```
src/robot/engineer/                         NUMBERS + WIRING (engineer-specific)
├── engineer_arm_constants.hpp                the joint specs: placement, axis, link mass, limits, gains, presets
├── engineer_actuator_constants.hpp           per-motor configs: Kt, gear ratio, direction, offset
└── engineer_control.cpp                      builds everything below and connects it   (exists)

src/control/arm/                            TAPROOT GLUE
├── arm_subsystem.hpp/.cpp                    thin Taproot wrapper   (exists)
├── arm_operator_input.hpp                    "what is the driver asking for" (interface)
├── remote_arm_operator_input.hpp/.cpp        remote + keyboard/mouse → that interface
├── leader_arm_input.hpp                      latest sample from the custom controller (interface)
├── arm_presets.hpp                           named poses: STOW, CARRY, PICK_ISLAND, ...
├── commands/                                 target-setters only
│   ├── arm_hold_command                        default command: stay where you are
│   ├── arm_home_command                        find zero on joints that need it, at startup
│   ├── arm_joint_jog_command                   move one joint with the stick
│   ├── arm_cartesian_jog_command               move the gripper in x/y/z
│   ├── arm_preset_command                      go to a named pose
│   └── arm_custom_controller_command           follow the leader arm
└── actuators/                                motor backends — the ONLY files that know about real motors
    ├── dji_joint_actuator                      M3508 / M2006 / GM6020
    ├── damiao_joint_actuator                   DM-J series in MIT mode
    └── coupled_wrist_actuator                  two motors ↔ two wrist joints (differential)

src/control/arm/core/                       PURE C++ (no Taproot — runs on robot, sim, and tests)
├── arm_types.hpp                             JointVector, Pose, Transform
├── joint_actuator_interface.hpp              the motor interface (JointActuatorInterface)
├── joint_config.hpp                          per-joint limits, PID gains, homing settings
├── arm_joint.hpp                             ArmJointSpec + ArmJoint — the building block
├── arm_model.hpp                             from the joint list: forward kinematics, gravity, Jacobian
├── ik/                                       pose → joint angles (closed-form or numeric)
├── limits/  geometry/                        joint ranges, keep-out zones, self-collision, tipping
├── arm_controller_interface.hpp              what commands are allowed to ask the arm to do
└── arm_controller.hpp                        ArmController<N> — runs the tick (header-only template)
```

> The subsystem is `ArmSubsystem`, not "six DOF arm subsystem": it doesn't know or care how many joints there
> are — the controller does.

Tests live in `test/control/arm/` (`arm_controller_test.cpp`, with `FakeJointActuator` standing in for motors).

---

## 4. ArmSubsystem vs. ArmController

Why two classes instead of one big subsystem?

| | `ArmSubsystem` | `ArmController<N>` |
|---|---|---|
| **What it is** | Taproot's view of the arm | The arm's brain |
| **Job** | Lets the command scheduler own the arm; ticks it every loop; handles remote disconnect | Reads joints, applies limits, plans motion, runs PD + gravity, sends torques |
| **Depends on** | Taproot (`tap::control::Subsystem`, the clock) | Plain C++ only |
| **Knows the joint count?** | No | Yes — the `N` |
| **Size** | ~30 lines | Almost all of the arm code |

The whole subsystem is about this much:

```cpp
void ArmSubsystem::refresh()
{
    float dt = /* time since last tick, from tap::arch::clock, clamped to a sane range */;
    controller.update(dt);
}

void ArmSubsystem::refreshSafeDisconnect()
{
    controller.enterSafeHold();   // HOLD the arm with gravity comp — never go limp (it would fall)
    controller.update(dt);
}
```

If everything lived in the subsystem instead:

1. **The simulator and unit tests couldn't run it easily** — a Taproot subsystem needs Taproot drivers. The
   controller is plain C++, so one file runs on the robot, in the sim, and in gtest with fake motors.
2. **It would become one giant class** that the whole subteam edits at once (Taproot plumbing + PID + gravity +
   IK + limits + homing).
3. **Commands would start reaching into the math** instead of just setting targets.

### 4.1 `ArmControllerInterface` — what commands can ask for

Commands and the subsystem hold an `ArmControllerInterface&`, never an `ArmController<N>`. The interface has no
joint count in its type, so a command works with any arm. Angles are rad, speeds rad/s, torques N·m at the joint.

| Method | Meaning |
|---|---|
| `initialize()` | one-time setup of every actuator |
| `update(dt)` | one control tick (called by the subsystem only) |
| `setJointTarget(angles, count)` | target for every joint; clamped to soft limits; ignored if `count` is wrong |
| `setJointTarget(index, rad)` | target for one joint (joint jog); clamped; ignored if out of range |
| `holdCurrent()` | target = where the arm is now |
| `enterSafeHold()` | remote lost: hold the pose captured on the first call; safe to call every tick |
| `disable()` | zero torque everywhere |
| `jointCount()` | number of joints |
| `jointPosition(i)` / `jointVelocity(i)` | latest feedback |
| `jointTarget(i)` | final target (after clamping) |
| `commandedTorque(i)` | torque sent on the last tick (logging, gravity tuning in §8.3) |
| `mode()` | `DISABLED`, `RUNNING` or `SAFE_HOLD` |
| `atTarget()` | every joint within its angle and velocity tolerance |
| `allOnline()` | every joint has fresh feedback |

`ArmController<N>` also has a typed `setJointTarget(const JointVector<N>&)` for code that knows `N` (the control
file, tests).

Not built yet — names reserved so the interface grows instead of changing:

| Build step | Methods |
|---|---|
| 3 (homing) | `isHomed()`, `setJointHomed(index, rad)`, plus a homing motion mode |
| 4 (arm model) | `setPayloadAttached(bool)` |
| 6 (IK) | `setPoseTarget(pose)`, `gripperPose()` |

---

## 5. What happens, step by step

### At startup (in `engineer_control.cpp`)

1. Create the **motor objects** (CAN bus + ID).
2. Wrap each motor in a **backend** (`DjiJointActuator`, `DamiaoJointActuator`, ...).
3. Pair each **joint spec** with its backend → six `ArmJoint`s.
4. Build the **`ArmController<6>`** from the joints, an IK solver, and the list of limits.
5. Build the **`ArmSubsystem`** and give it the controller.
6. Build the **commands** and give them the subsystem.
7. Register the **key/switch mappings**. Schedule **homing** once. Set **hold** as the default command.

### Every tick (500 Hz)

1. Taproot calls `ArmSubsystem::refresh()`.
2. That calls `controller.update(dt)`.
3. The controller:
   1. reads every joint's angle and speed,
   2. runs the current target through the limits and the trajectory planner → *"where should each joint be right now, and how fast"*,
   3. computes each joint's torque (PD + gravity, §7),
   4. clamps it to the joint's and the motor's limits,
   5. calls `commandTorque()` on each joint.
4. Each backend turns its torque into a motor command.

### When the driver presses a preset key

1. The mapping schedules `ArmPresetCommand`.
2. Its `initialize()` calls `controller.setJointTarget(presetPose)`. **That's all the command does.**
3. Over the next ticks, the controller moves the arm there — safely, through the limits.
4. The command's `isFinished()` asks `controller.atTarget()`.

---

## 6. Joints and actuators

Three pieces:

```
ArmJointSpec    what the joint IS         pure data, no motor
ArmJoint        spec + JointActuator*     the pairing; built in the control file
JointActuator   the motor interface       "put this torque on the joint" / "where is the joint"
```

### 6.1 `ArmJointSpec` — the joint as data

Everything about a joint **except** the motor. It's a `constexpr` table in `engineer_arm_constants.hpp`.
The simulator generates the same numbers from CAD, so robot and sim always agree.

```cpp
struct ArmJointSpec {
    const char*  name;          // "shoulder_pitch"
    JointType    type;          // REVOLUTE (rotates) or PRISMATIC (slides)
    JointOrigin  origin;        // where it sits relative to the previous joint
    float        axis[3];       // which way it rotates
    LinkInertia  link;          // mass + centre of mass of the part after this joint (for gravity)
    Capsule      collision[2];  // simple shapes for collision checks
    JointConfig  config;        // angle range, max speed, max torque, PD gains
    HomingConfig homing;        // how to find zero (or NONE if the encoder is absolute)
};
```

### 6.2 `ArmJoint` — spec + motor

```cpp
struct ArmJoint {
    const ArmJointSpec* spec;
    JointActuatorInterface* actuator;   // points at a backend; doesn't own the motor
};
```

### 6.3 `JointActuator` — the motor interface

In code this is `JointActuatorInterface` (`core/joint_actuator_interface.hpp`); this doc says `JointActuator`
for short.

The **only** place a motor exists, as far as the arm is concerned. Everything is in **joint units** — radians and
N·m at the joint. Gear ratios, direction flips, encoder offsets and CAN details stay hidden inside the backend.

| Method | Meaning | Units |
|---|---|---|
| `initialize()` | one-time setup (e.g. send an enable frame) | — |
| `update()` | pull the latest feedback | — |
| `position()` | joint angle | rad |
| `velocity()` | joint speed | rad/s |
| `measuredTorque()` | torque at the joint (reported or estimated) | N·m |
| `online()` | feedback is fresh | bool |
| `absolute()` | knows its true angle at power-up? `false` → needs homing | bool |
| `setCurrentPositionAs(rad)` | homing: "the joint is at this angle right now" | rad |
| `limits()` | peak / continuous / available torque, max speed, thermal headroom | N·m, rad/s, 0–1 |
| `commandTorque(τ)` | apply this torque at the joint | N·m |
| `disable()` | zero output | — |

### 6.4 Why the joint *points at* a motor instead of *being* one

The tempting design is `class DamiaoJoint : public Joint`. It breaks on the wrist.

The draft wrist is a **differential**: two motors drive wrist pitch and wrist roll *together*. Both motors turn the
same way → the wrist pitches. Opposite ways → it rolls. Neither motor belongs to just one joint, so "a joint is a
kind of motor" can't describe it.

With the pointer design, `CoupledWristActuator` owns both motors and hands out **two** `JointActuator` views — one
that looks like the pitch joint, one like the roll joint. The arm sees six ordinary joints.

Other wins:

- **Swapping a motor** = change one actuator object in the control file.
- **Tests and the sim plug in their own actuators** (`FakeJointActuator`, a sim motor) — the arm code is identical.
- **Specs stay pure data** — `constexpr`, generated from CAD, shared with the sim.

---

## 7. The control loop

> **Status:** recommended answer to the open "cascade vs. PD" question. The math track confirms.

### 7.1 One loop per joint: PD + gravity

Each joint's torque is three parts added together:

| Part | What it does | Comes from |
|---|---|---|
| **Gravity hold** | the torque needed to keep the arm still at its current pose | the arm model (§8) — no error involved |
| **Spring (P)** | the further the joint is from where it should be right now, the harder it pushes back | `kp × angle error` |
| **Damper (D)** | resists the gap between how fast it's moving and how fast it *should* be moving | `kd × speed error` |

**It's a position loop whose output is a torque.** The input is position (and speed) error; the gains turn that
into N·m. This is the same idea as the turret — angle error in, motor output out — except the output here is
*joint torque*, and the conversion to motor current happens one step later, in the backend.

### 7.2 Worked example (made-up numbers)

The shoulder, mid-move:

| Step | |
|---|---|
| The planner says | be at **0.80 rad**, moving **0.5 rad/s** |
| The motor says | at **0.70 rad**, moving **0.3 rad/s** |
| **P** | 0.10 rad behind × kp 40 N·m/rad = **4.0 N·m** |
| **D** | 0.2 rad/s too slow × kd 3 N·m/(rad/s) = **0.6 N·m** |
| **Gravity** | the model says this pose needs **12.0 N·m** to hold |
| **Total** | **16.6 N·m** → clamp to limits → `commandTorque(16.6)` |
| **Backend** | DJI: ÷ gear ratio ÷ Kt → amps → scale to ±16384 → `setDesiredOutput`. Damiao: straight into the frame's torque slot. |

When the arm is sitting still at its target, P and D are near zero — **gravity is what holds it up**.

### 7.3 Moving vs. holding

- **Gravity depends on pose, not direction.** It's recomputed every tick from the joint angles. Swinging toward
  vertical, it shrinks on its own; it doesn't matter whether the arm got there going up or down.
- **Moving up** needs gravity *plus* a bit extra to accelerate. That extra comes from PD — the joint lags the
  plan slightly, so P and D push harder.
- **Moving down**, gravity helps; PD comes out negative and holds the arm back so it follows the plan instead of falling.
- **PD covers everything gravity doesn't**: accelerating, friction, and small errors in the gravity model.

### 7.4 Design choices

- **D uses the planned speed, not zero**, so the joint follows a moving target without lagging.
- **Integral is optional.** If used: small, clamped, reset when the target jumps. Gravity comp already does the
  job an integral usually does on an arm (fight the sag); a big integral overshoots and winds up.
- **Not a cascade loop** (position → velocity → torque, like chassis/turret). That's twice the gains per joint and
  tends to fight the gravity term.
- **Not full "computed torque" (yet).** Each joint is controlled on its own; only gravity is shared. If fast moves
  are sloppy, *inertia feed-forward* can be added later — an add-on, not a redesign.
- **The differential wrist** runs this loop on pitch and roll; its backend mixes the two torques into two motors.

### 7.5 Why output torque instead of a raw motor value?

The turret feeds its PID straight into `setDesiredOutput`. That's fine for one motor on one mechanism. For the arm:

- **Gains mean the same thing on any motor.** `kp` is "N·m per radian". Swap a motor or gearbox → the gains still hold.
- **Gravity is a torque**, so it has to be in N·m anyway.
- **The limits are torques** — motor caps, "can it hold this pose without overheating", the wrist's shared budget.
- **The differential wrist mixes torques.**
- **Voltage commands give less torque as the motor speeds up** (back-EMF). Torque (current) commands don't.
- **The simulator models torque**, so robot and sim run the same loop.

---

## 8. Gravity compensation

### 8.1 From kG to the whole arm

For a **one-joint** arm, gravity comp is the familiar `kG × cos(angle)` (WPILib's `ArmFeedforward`).

That **doesn't work per joint on a multi-joint arm**, because the torque a joint needs depends on where
*everything past it* is — not just its own angle:

```
  Arm stretched out                     Same shoulder angle, elbow folded back
  shoulder ●━━━━━━━━●━━━━━━━━●          shoulder ●━━━━━━━━●
                                                           ┃
           forearm is FAR out                              ●   forearm is CLOSE in
           → shoulder works hard                               → shoulder works much less
```

Same shoulder angle, different torque. `kG × cos(own angle)` can't tell these apart, and it can't account for an
ore in the gripper either.

**The general version applies kG per *link* instead of per *joint*:**

> For each joint, add up — for every link beyond it — **that link's weight × how far out horizontally it sits from
> this joint's axis**.

"How far out horizontally" depends on all the joint angles, and the arm model already computes it. That's
`ArmModel::gravityTorques()`. For a one-joint arm it reduces exactly to `kG × cos(θ)`.

### 8.2 Things that fall out for free

- **J1 (base yaw)** turns about a vertical axis → gravity torque is always **zero**.
- **Roll joints** only feel gravity if what they carry is off-centre.
- **Carrying an ore:** `setPayloadAttached(true)` adds the ore's mass at the gripper, and every joint's hold torque
  goes up automatically.

### 8.3 Tuning it

Link masses start from CAD, which is never exact (wires, bolts, the ore). Two ways to correct:

1. **Per-link mass scale** — a kG-style multiplier per link. If the arm sags, bump that link's factor until it holds.
2. **Measure it** — hold the arm still at several poses, record each joint's torque, and fit the link masses to match.
   The simulator can pick poses that tell the masses apart.

Good gravity comp means PD barely works to hold a pose → soft gains, small or no integral.

---

## 9. Motor backends

The files in `actuators/` are the **only** files that include motor headers. CAN IDs are set where the motor objects
are created, in the control file.

### 9.1 Overview

| Backend | Motors | Absolute at power-up? | Sent by |
|---|---|---|---|
| `DjiJointActuator` | M3508 (C620), M2006 (C610), GM6020 | GM6020 yes · M3508/M2006 **no** (encoder is before the gearbox) | existing `djiMotorTxHandler` flush in the robot loop |
| `DamiaoJointActuator` | DM-J8009P-2EC, DM-J4340-2EC, ... | 2EC (dual encoder) should be — **verify** | its own send step (to add) |
| `CoupledWristActuator` | two motors → wrist pitch + roll | depends on the motors (M2006 → no) | through its two inner backends |

Torque ↔ motor-unit conversions live in small free functions (e.g. `dji_torque_conversion.hpp`) so the simulator's
motor model can run the same math in reverse.

### 9.2 Draft joint map (⚠️ not decided)

| Joint | Draft motor | Backend | Homing |
|---|---|---|---|
| J1 base yaw | TBD (maybe M3508) | `DjiJointActuator` | needed, unless an external encoder is added |
| J2 shoulder pitch | DM-J8009P-2EC | `DamiaoJointActuator` | none if absolute (verify) |
| J3 elbow pitch | DM-J4340-2EC | `DamiaoJointActuator` | none if absolute (verify) |
| J4 forearm roll | GM6020 | `DjiJointActuator` | none (absolute single-turn) |
| J5 + J6 wrist pitch/roll | 2× M2006, differential | `CoupledWristActuator` | needed (hard stop or limit switch, TBD) |

If any of these change, only the control file and `engineer_actuator_constants.hpp` change.

### 9.3 DJI: torque → `setDesiredOutput`

`setDesiredOutput()` is **not torque**. It's a raw integer the motor driver reads as a **current** or a **voltage**:

| Motor (driver) | `setDesiredOutput` means | Taproot range |
|---|---|---|
| M3508 (C620) | current | ±16384 (`MAX_OUTPUT_C620`) |
| M2006 (C610) | current | ±10000 (`MAX_OUTPUT_C610`) |
| GM6020, default | **voltage** | ±25000 (`MAX_OUTPUT_GM6020`) |
| GM6020, current mode | current | ±16384 (`MAX_OUTPUT_GM6020_mA`) |

`DjiJointActuator::commandTorque(τ)` converts:

1. **Joint torque → motor torque:** ÷ gear ratio ÷ efficiency.
2. **Motor torque → amps:** ÷ the motor's torque constant Kt (datasheet).
3. **Amps → integer:** scale by the driver's range (e.g. C620: ±16384 ↔ ±20 A — check the datasheet).
4. Apply the direction sign, clamp, call `setDesiredOutput`.

Torque is very nearly proportional to current, and the C620/C610 run their own fast current loop — so a current
command gives the intended torque at any speed.

#### GM6020 rules for the arm

- ✅ **Run it in current mode.** Construct its `DjiMotor` with `currentControl = true` (default `false`).
  In voltage mode the torque you get drops as the joint speeds up.
- ✅ **No setup tool needed for the mode.** It's picked per CAN frame: **0x1FF = voltage**, **0x1FE = current**.
  With `currentControl = true`, Taproot's `DjiMotorTxHandler` puts the motor in the 0x1FE frame automatically.
- ⚠️ **RoboMaster Assistant is only for firmware.** Current mode needs newer GM6020 firmware; older motors ignore
  0x1FE. If a current-mode GM6020 sends feedback but won't move, update its firmware. Check DJI's release notes for
  the minimum version before the first bench test.
- ⚠️ **DIP-switch ID must be 1–4** (Taproot `MOTOR5`–`MOTOR8`). Taproot only sends 0x1FE, which covers GM6020s 1–4.
- ⚠️ **Command frames are shared per bus.** A GM6020 and an M3508/M2006 on the same bus can't both use DJI ID 5–8.
  Put this in the CAN ID map.
- ⚠️ **Verify the current-mode scale.** Taproot's comment says "mA"; DJI's manual maps ±16384 to about ±3 A.
  Use the datasheet.
- ❌ **Don't copy the turret's setting.** The turret runs GM6020s in voltage mode — fine there, wrong for the arm.

### 9.4 Damiao: MIT mode as a torque motor

In **MIT mode**, a Damiao takes one CAN frame with five values: target position, target velocity, stiffness (kp),
damping (kd), and a feed-forward torque. The motor runs its own little control law with them.

**Plan: use it as a pure torque motor.** Send kp = 0, kd = 0, target position/velocity = 0, and put our torque in
the feed-forward slot. Then every joint on the arm behaves the same way:

- one kind of loop to tune, in one place (the controller),
- one motor model shape in the simulator,
- gravity and limits stay in our code where we can see and log them.

What's different inside the backend:

| Topic | What to do |
|---|---|
| **Feedback scaling** | Frames pack position/velocity/torque as scaled integers over the ranges set on the motor (PMAX / VMAX / TMAX). Firmware must use the **same** ranges or every reading is wrong. Take them from the motor's config, not from memory. |
| **Output-side feedback** | These are gear motors that report the output shaft, so the gear ratio is usually 1 from the joint's view. Verify per model. |
| **Absolute position** | Dual encoder → `absolute()` returns `true` and homing skips the joint. **Bench-test** across power cycles first. |
| **Enable / disable** | Needs an explicit enable frame in `initialize()` and a disable path in `disable()`. |
| **Send path** | Not part of the DJI flush. The robot loop needs its own Damiao send step (done once, not per joint). |
| **CAN IDs** | Keep Damiao IDs clear of the DJI range 0x200–0x20B. |
| **`limits()`** | Peak / continuous torque and max speed from the datasheet. Damiao feedback includes temperature, which helps the thermal estimate. |

**Possible later upgrade (not now):** torque-only throws away MIT mode's best feature — the motor can close the
position loop itself, faster than our 500 Hz over CAN, which usually means a stiffer joint. If J2/J3 feel soft, add
an optional "position target + gains, close the loop yourself" method to `JointActuator`. DJI backends ignore it,
Damiao uses it, and the controller still sends gravity as the feed-forward.

### 9.5 Coupled (differential) wrist

- One object owns **both** motors and exposes **two** `JointActuator` views (pitch, roll).
- Inside, it converts joint torques ↔ motor torques and motor angles ↔ joint angles through the differential's
  2×2 mixing.
- **The two joints share torque capacity.** If pitch is using most of both motors, roll has less left, so one view's
  `limits()` depends on what the other is commanding. The limiter has to know (math-track TODO).
- M2006s aren't absolute → the wrist needs homing (hard stop or limit switch, TBD).

---

## 10. Build order

The full layout is a lot. It splits into steps that each **run on the bench**:

| Step | Build | You can now... |
|---|---|---|
| **1** | `JointActuator` interface + `DjiJointActuator` | command a small torque on one motor; confirm direction and size |
| **2** | `ArmSubsystem` (rename) + `ArmController` with **joint-space PD + gravity** and angle clamps only. Hold + preset commands. | hold the arm and send it to fixed poses |
| **3** | Joint jog + homing commands; `RemoteArmOperatorInput` | drive the arm joint by joint |
| **4** | `ArmModel`: real forward kinematics + gravity from the joint list; check in the sim | trust the gravity comp and see the gripper pose |
| **5** | The limits list: keep-out zones, self-collision, tipping | move without hitting the robot or the floor |
| **6** | IK + Cartesian jog | move the gripper in x/y/z |
| **7** | `ArmCustomControllerCommand` | drive the arm with the leader arm |

Steps 1–3 give a working arm with **none of the hard math**. Every later step plugs in behind the same controller
interface, so the subsystem and the commands don't change.

---

## 11. Checklist: adding or swapping a motor

1. **New motor family?** Write a backend in `actuators/` that implements every `JointActuator` method in joint units,
   plus its config struct. If it has its own CAN send path, add that to the robot loop (once).
2. **Config:** add Kt / ranges, gear ratio, direction, offset and limits to `engineer_actuator_constants.hpp`, from
   the datasheet. For a GM6020: `currentControl = true`.
3. **Wire it:** in `engineer_control.cpp`, create the motor object (bus + ID) and the backend, and pair it with the
   joint's spec in the `ArmJoint` array.
4. **Homing:** if the joint changed between absolute and relative, update its `HomingConfig`.
5. **Bookkeeping:** update the CAN ID map and the simulator's motor YAML for that joint.
6. **Sanity check:** nothing in `core/`, the subsystem, or the commands should need to change. If it does, the backend
   is leaking motor details — fix the backend instead.

---

## 12. Glossary

| Term | Meaning |
|---|---|
| **DOF** | Degrees of freedom — how many joints. The engineer arm has 6. |
| **Joint space** | Describing the arm by its joint angles (`[j1, j2, ... j6]`). |
| **Cartesian / task space** | Describing the gripper by where it is (x, y, z) and how it's tilted. |
| **Forward kinematics (FK)** | Joint angles → where the gripper is. Always one answer. |
| **Inverse kinematics (IK)** | Where you want the gripper → joint angles. May have several answers, or none. |
| **Jacobian** | How fast the gripper moves for a small change in each joint. Used for Cartesian jog and limits. |
| **Feed-forward** | Torque you add because you *know* you'll need it (e.g. gravity), not because of an error. |
| **PD** | Proportional + derivative control: push toward the target (P), damp the motion (D). |
| **Backend** | A class that implements `JointActuator` for one kind of motor. |
| **Absolute encoder** | Knows the real angle at power-up. Relative ones need homing. |
| **Homing** | Moving a joint to a known spot (hard stop, switch) at startup to find its zero. |
| **MIT mode** | Damiao's control mode: position, velocity, kp, kd and torque in one frame. |
| **Back-EMF** | The voltage a spinning motor generates that pushes back against the supply — why voltage ≠ torque. |
| **Kt** | Torque constant: N·m of torque per amp of current. |
| **Keep-out zone** | A region the arm is not allowed to enter (chassis body, floor, ...). |

---

## 13. Open questions

**Hardware (mechanical)**
- J1 motor and how it's zeroed (external encoder? homing?); the beyblade / slip-ring question.
- Differential wrist: bevel ratio, homing method, and whether the J4 axis passes through the wrist centre
  (the closed-form IK needs it).

**Bench tests**
- Do the 2EC Damiaos really report absolute output angle across power cycles?
- Damiao PMAX / VMAX / TMAX values to configure and match in firmware.
- GM6020: current-mode scale (A per count), minimum firmware for current mode, and what the team's motors are on.

**Controls**
- Math track confirms PD + gravity (§7) over cascade; does any joint need a small integral?
- Is torque-only MIT mode stiff enough for J2/J3, or is the motor-side PD upgrade (§9.4) needed?
- How link masses get calibrated: per-link scale vs. fitting from measured holding torques (§8.3).
