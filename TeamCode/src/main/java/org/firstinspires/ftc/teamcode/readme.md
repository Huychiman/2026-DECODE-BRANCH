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
| `src/test/` | 56 JUnit tests including a full 30 s autonomous physics simulation | PC |

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
| Audience | Back bumper against the **audience wall**, 12 in from the centre line on our side, facing the goals, straddling the small launch zone line. Shoots from about 4.5 in off the wall, leaves to about (34, 26). |

Coordinates are FTC SDK field inches (origin field centre, goals at -X, Red +Y). Rule checks
(start over a launch line touching our goal or the wall on our half, launch only from a zone, end
outside both zones) are automated in `FieldRulesTest` and `AutoSimulationTest`.

## Checked against the CAD

Numbers below come from the Fusion assembly "Robot" (project BFC 2025-2026) and are pinned by
`RobotConfigCadTest`.

| Item | CAD | Code |
|---|---|---|
| Drive | REV HD Hex + UltraPlanetary 4:1 (76:21) + 5:1 (68:13), 22T:22T HTD belt, 104 mm mecanum | 530.05 ticks/wheel rev, 0.024268 in/tick (was a goBILDA 312 rpm guess, 1.4% off) |
| Footprint | 45.7 x 48.1 cm = 18.0 x 18.9 in | `ROBOT_WIDTH` / `ROBOT_LENGTH`; audience start moved so the back touches the wall |
| Odometry | goBILDA Pinpoint + two 4-bar dead-wheel pods | Used automatically when configured as `pinpoint`; falls back to motor encoders + IMU |
| Limelight | lens 8.7 in high, 7.7 in forward, **3.7 in right** of the shooter | Aim corrects tx for the offset (without it goal-start shots were ~5 deg off in simulation) |
| Shooter | HD Hex, 30T:30T belt, four 90 mm grip wheels; hood servo 24T on a 264T sector (~24.5 deg travel) | Speeds stay in motor ticks/s, so no change |
| Sorter | 38T gear driving 20T on a 3-cup carousel (120 deg per cup) | Kept the competition positions 0.10/0.50/0.90; see the warning below |

Things the CAD shows that need a human decision:

* **Length 18.9 in** is over the 18 in starting size limit. The extra is the intake roller (3.3 cm out
  the front) and the cable channel (1.8 cm out the back). Measure the real robot before inspection.
* **Sorter gearing vs. servo positions.** With a 270 deg REV Smart Servo on the 38T gear, one cup is
  0.234 of servo travel; the competition code uses 0.40. Either the real servo or gearing differs from the
  CAD, or the cups only lined up approximately. Check with *Tune: Servos*.
* The Limelight in the CAD is not rotated (lens facing straight up), so its tilt is not known from CAD.
  Its height and offsets are used; its tilt is not needed because range comes from the 3D tag pose.

## Tuning checklist: do these in order before the next event

Values marked `TUNE` in `RobotConfig` are defaults that have **not** been measured on this robot.

1. **Tune: Drive Motors**: hold each face button. Every wheel must push forward and its count go up.
   Then press Share and push the robot exactly 48 in: *inches per tick* should read about 0.0243 (the
   CAD value). Strafe 48 in by hand to set `LATERAL_MULTIPLIER`.
2. **Tune: Pinpoint** (if the Pinpoint is wired): add it to the configuration as
   "goBILDA Pinpoint Odometry Computer" named `pinpoint`. Push forward 48 in (X must read +48), left 48 in
   (Y must read +48), then spin 10 turns in place (X and Y must stay within about 1 in). Flip
   `PINPOINT_X/Y_REVERSED` or fix the offsets if not. Auto and TeleOp telemetry show which pose source
   is in use.
3. **Tune: Servos**: confirm `SORTER_POSITIONS`, `KICKER_REST` / `KICKER_FIRE` and the hood range.
4. **Tune: Shooter, step 1 (most important)**: hold Square (0.8 power, the competition shot) until the
   speed settles and copy *kV estimate* into `flywheelGains().kV`. Until you do, closed-loop control
   aims for 2240 ticks/s, which is only the competition shot if the wheel really reached 2240 at 0.8
   power. If shots fly long or short before tuning, this is why.
5. **Tune: Shooter, step 2**: at 4-6 distances across both launch zones, hold RB, adjust speed and hood
   until LB shots score, and copy each `ROW` line into `shotTable()`. The Audience auto is a long shot
   (about 125 in) and needs a real row out there.
6. **Tune: Limelight**: confirm pipeline 8 sees tags 20/24 and 21-23. Height and mount offsets come
   from the CAD; re-measure them if the camera mount changed.
7. Measure the real footprint against `ROBOT_LENGTH` / `ROBOT_WIDTH` (CAD: 18.9 x 18.0 in) and re-run
   the tests.

## Tests and simulation

```
./gradlew :TeamCode:testDebugUnitTest
```

Runs on a laptop, no robot needed. `AutoSimulationTest` drives the real `AutoRoutine`, odometry and
flywheel controller against a physics model (motor lag, strafe slip, encoder quantisation, IMU drift,
battery sag, per-shot flywheel speed loss, servo travel, noisy delayed Limelight, 12-28 ms loop jitter).
Wheel speed, turning radius, footprint and the off-centre Limelight come from the CAD.
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
* Odometry: uses the goBILDA Pinpoint when present; motor encoders are now scaled for the real REV
  drivetrain.
* Auto-aim corrects for the Limelight sitting 3.7 in right of the shooter.
* Flywheel: fixed 0.8 power changed to battery-compensated closed-loop speed, with an automatic
  open-loop fallback if its encoder is unplugged.
