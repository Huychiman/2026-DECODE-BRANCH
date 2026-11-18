# DECODE robot code

NextFTC 1.1 + FTC SDK 11.0. Hardware names are unchanged from the first-competition code, so the
existing Driver Station configuration works as is.

## Layout

| Package | What | Runs on |
|---|---|---|
| `logic/` | All decisions: mecanum maths, odometry, flywheel control, shot table, motif planner, shot sequencer, field rules, autonomous state machine | robot **and** PC (pure Java, unit tested + simulated) |
| `config/RobotConfig` | Every hardware name and tunable number | both |
| `subsystems/` | Thin hardware wrappers (NextFTC `Subsystem`s): Drive, Intake, Sorter, Kicker, Shooter, Vision | robot |
| `opmodes/` | TeleOp (Main/Red/Blue), Auto (Red/Blue x Goal/Audience), `tuning/` OpModes | robot |
| `src/test/` | 50 JUnit tests including a full 30 s autonomous physics simulation | PC |

## Driver controls (gamepad1; gamepad2 can also run every mechanism button)

| Input | Action |
|---|---|
| Left stick / right stick X | drive, strafe / turn (identical wheel output to the competition code — proven by `MecanumTest`) |
| Left trigger | slow mode (40%) |
| Right trigger | auto-aim at the goal with the Limelight (you keep translating) |
| Triangle | intake on/off |
| Cross (hold) | eject |
| Square / Circle | sorter previous / next slot |
| Right bumper (hold) | flywheel on (speed from the shot table when the goal tag is visible) |
| Left bumper (hold) | kicker fire (manual single shot) |
| D-pad up / down | hood trim |
| D-pad left | **fire all 3 slots automatically** (waits for flywheel speed between shots) |
| D-pad right | cancel the volley |
| Share | toggle field-centric (starts robot-centric) |
| Options | reset field-centric "forward" to where the robot faces now |

Use **TeleOp Red / TeleOp Blue** so auto-aim ignores the other alliance's goal. The autos preselect them.

## Autonomous

Each auto: drive to the shot spot while the flywheel spins up, aim (Limelight tx, odometry fallback),
fire the 3 preloads in the obelisk MOTIF order, then LEAVE. Simulated: done in about 5-6 s.
It gives up shooting at 24 s, whatever happens, so it still scores LEAVE.

Load the sorter **slot 0 = GREEN, slot 1 = PURPLE, slot 2 = PURPLE** (`RobotConfig.PRELOAD`).
The motif is read from the obelisk tags (21 GPP, 22 PGP, 23 PPG). If the motif is never seen, the auto
fires slots in order.

| Auto | Start placement (Red shown; Blue is the mirror image) |
|---|---|
| Goal | Front bumper flat against the front face of **our goal**, robot centred on the goal face, facing the goal, straddling the launch line. Backs up to about (-30, 24), shoots, leaves to about (-6, 40). |
| Audience | Back bumper against the **audience wall**, 12 in from the centre line on our side, facing the goals, straddling the small launch zone line. Shoots from 5 in off the wall, leaves to about (34, 26). |

Coordinates are FTC SDK field inches (origin field centre, goals at -X, Red +Y). Rule checks
(start over a launch line touching our goal or the wall on our half, launch only from a zone, end
outside both zones) are automated in `FieldRulesTest` and `AutoSimulationTest`.

## Tuning checklist: do these in order before the next event

Values marked `TUNE` in `RobotConfig` are defaults that have **not** been measured on this robot.

1. **Tune: Drive Motors**: hold each face button. Every wheel must push forward and its count go up.
   Then press Share, push the robot exactly 48 in, copy *inches per tick* into `DRIVE_INCHES_PER_TICK`.
   Strafe 48 in by hand to set `LATERAL_MULTIPLIER`.
2. **Tune: Servos**: confirm `SORTER_POSITIONS`, `KICKER_REST` / `KICKER_FIRE` and the hood range.
3. **Tune: Shooter, step 1 (most important)**: hold Square (0.8 power, the competition shot) until the
   speed settles and copy *kV estimate* into `flywheelGains().kV`. Until you do, closed-loop control
   aims for 2240 ticks/s, which is only the competition shot if the wheel really reached 2240 at 0.8
   power. If shots fly long or short before tuning, this is why.
4. **Tune: Shooter, step 2**: at 4-6 distances across both launch zones, hold RB, adjust speed and hood
   until LB shots score, and copy each `ROW` line into `shotTable()`. The Audience auto is a long shot
   (about 125 in) and needs a real row out there.
5. **Tune: Limelight**: confirm pipeline 8 sees tags 20/24 and 21-23, and set `LIMELIGHT_HEIGHT_IN`.
6. Measure the bumper footprint into `ROBOT_LENGTH` / `ROBOT_WIDTH` and re-run the tests.

## Tests and simulation

```
./gradlew :TeamCode:testDebugUnitTest
```

Runs on a laptop, no robot needed. `AutoSimulationTest` drives the real `AutoRoutine`, odometry and
flywheel controller against a physics model (motor lag, strafe slip, encoder quantisation, IMU drift,
battery sag, per-shot flywheel speed loss, servo travel, noisy delayed Limelight, 12-28 ms loop jitter).
It covers every alliance x start x motif, 40 random seeds, no Limelight, an unplugged flywheel encoder,
a heavy slow flywheel, a low battery with heavy strafe slip, and a pinned robot.
Traces go to `TeamCode/build/sim/*.csv`; `python TeamCode/tools/plot_sim.py` draws them on the field
(see `TeamCode/docs/auto-sim.svg`).

Each change to tunables should be followed by a test run: the rule checks catch illegal paths (they
already caught one: the first audience shot spot let the corners swing into the wall).

## Changes from the first-competition code

* It compiles again: NextFTC was not declared as a dependency, and `TeleOpMain` imported a class that
  did not exist.
* `Shooter` and `Picker` both drove `pickerServo`; now only `KickerSubsystem` does.
* Drive motor directions are flipped with the opposite sign in the maths, so the wheels get exactly the same
  output, and odometry now reads forward as positive.
* Flywheel: fixed 0.8 power changed to battery-compensated closed-loop speed, with an automatic
  open-loop fallback if its encoder is unplugged.
