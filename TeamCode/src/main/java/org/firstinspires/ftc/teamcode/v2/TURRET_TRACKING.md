# Turret Tracking — V2 Analysis & Limitations

---

## 0. Turret Speed — V1 vs V2

### V1 — GoBilda 312 RPM Yellow Jacket Motor

```
Motor free speed:        312 RPM
Gear train:              30t motor gear → 90t turret ring = 3:1 reduction
Turret output speed:     312 × (30/90) = 104 RPM
                         104 × 360°/60s = 624°/s
Wrap 360° time:          360° / 624°/s = 0.58s
Overcurrent protection:  None (DC motor controller)
Position feedback:       Motor encoder (real position every loop)
```

### V2 — Axon MAX × 3 Servos

```
Servo:                   Axon MAX MK2
Servo hub voltage:       6V (REV Servo Hub default — confirmed)
Axon MAX speed @ 6V:     0.115s/60° = 521°/s servo shaft
Gear train:              48t servo → 15t idler → 47t drives 107t turret ring
                         Ratio = (48/15) × (47/107) = 1.408x speed INCREASE
Turret output speed:     521°/s × 1.408 = 733°/s theoretical
                         733°/s / 360° × 60 = 122 RPM theoretical
```

**Measured (TurretTest — 0° to 180°):**
```
Total time observed:     0.40s – 0.60s (stopwatch)
Breakdown:               ~0.25s actual travel + ~0.15s overshoot/correction
Effective travel speed:  180° / 0.25s = 720°/s ≈ theoretical 733°/s ✓
Wrap 360° travel time:   360° / 720°/s = 0.50s
Settled time (with overshoot): ~0.60s
```

The servos are NOT slow under load — the mechanism has sufficient torque.
The gap between 0.25s travel and 0.60s total is overshoot and correction only.

### Comparison

| | V1 (312 RPM motor) | V2 (Axon MAX × 3) |
|---|---|---|
| Theoretical speed | 624°/s (104 RPM) | 733°/s (122 RPM) |
| Effective travel speed | ~624°/s | ~720°/s |
| Wrap 360° travel time | 0.58s | **0.50s** |
| Settled time (with overshoot) | ~0.58s | ~0.60s |
| Overcurrent protection | None | Yes (servo hub beep) |
| Position feedback | Motor encoder | Abs encoder (4th wire, not yet wired) |

**V2 is faster than V1.** The only disadvantage is servo hub overcurrent protection
and lack of position feedback — both addressed by wiring the 4th wire.

---

## 1. Wrap-Around: Why Tracking Is Slow and How to Test

### Background

The turret must continuously point at the goal while the robot drives at any heading.
The target angle in robot-frame is calculated each loop:

```
targetAngleDeg = atan2(goalY - robotY, goalX - robotX)° - robotHeading°
```

`atan2` outputs [-180°, +180°]. `robotHeading` spans [-180°, +180°].
So `targetAngleDeg` can range across **[-360°, +360°]**.

The turret physical range is **-160° to +265°** (425° total, software limited).
Since 425° > 360°, every possible goal bearing is always reachable — but not always from
the same side. When `targetAngleDeg` drifts past a limit, the candidate picker must
jump to the equivalent angle on the opposite side (~360° away).

### Why the Jump Is Slow

The turret effective travel speed is **~720°/s** (see Section 0).
A full 360° wrap jump takes:

```
360° / 720°/s = 0.50s travel
+ ~0.10s overshoot/correction = ~0.60s total settled
```

During that 0.60s the turret cannot shoot accurately.

### The Old Problem (Fixed)

The previous code normalized `targetAngleDeg` to [-180°, +180°] using `atan2(sin, cos)`.
This created an **artificial discontinuity at ±180°** — completely unrelated to the physical
turret limits. The jump could trigger at any robot heading, unpredictably.

When the jump happened while the turret was already moving in the opposite direction:
- Servo was mid-travel at ~422°/s shaft speed
- New command reversed direction instantly
- Servo hub detected overcurrent → **beeped and cut power** (1-2 second recovery)

### The Fix

Normalization removed. `targetAngleDeg` is now unbounded, matching V1 behavior.
Wrap only triggers at the genuine physical limits (-160°/+265°), not at ±180°.

### Remaining Wrap Behavior

| Scenario | Jump magnitude | Travel time | Settled time | Risk |
|---|---|---|---|---|
| Turret near 0°, target crosses -160° | ~360° | ~0.50s | ~0.60s | Low — servo moving slowly |
| Turret near -150°, target crosses -160° | ~349° | ~0.48s | ~0.58s | **High — direction reversal** |
| Turret near +230°, target crosses +265° | ~349° | ~0.48s | ~0.58s | **High — direction reversal** |
| Normal tracking (no wrap) | <30° | <0.04s | <0.1s | None |

Direction reversal is the overcurrent risk. Without abs encoder feedback, we cannot
detect whether the servo is mid-travel before issuing a reversal command.

### LIMIT_WRAP_JUMP Flag (Current Mitigation)

```java
public static boolean LIMIT_WRAP_JUMP = false;   // enable on Dashboard
public static double MAX_WRAP_JUMP_DEG = 90.0;   // °/loop during wrap
```

When enabled: wrap jump is spread over multiple loops (90°/loop).
Each loop recalculates from current robot heading — tracks the moving target continuously.

| Loop | lastChosenDeg | chosen (raw) | After limiter | Servo commanded |
|---|---|---|---|---|
| 1 | -150° | +199° (349° jump) | -150° + 90° = **-60°** | -60° |
| 2 | -60° | +197° (257° jump) | -60° + 90° = **+30°** | +30° |
| 3 | +30° | +195° (165° jump) | **+195°** (< 180°, no limit) | +195° |

Total: 3 loops × 20ms = **60ms** to complete wrap. No direction reversal. No overcurrent.

If robot reverses heading during loop 2: candidate picker recalculates from new heading,
picks the near candidate on the same side as `lastChosenDeg` — turret snaps back correctly.

### How to Test

1. Run `TeleopMoving`, filter logcat: `adb logcat -s Turret`
2. Spin robot continuously — watch for `WRAP JUMP` warnings in logcat
3. Listen for servo hub beep

**Test matrix:**

| LIMIT_WRAP_JUMP | Expected result |
|---|---|
| false | Wrap jump in 1 loop — may beep if servo was mid-travel |
| true | Wrap spread over ~3 loops — no beep, 60ms tracking gap |

Watch Dashboard graphs simultaneously:
- `TurretTargetDeg` — raw atan2 result, should change smoothly
- `TurretDeg` — commanded position, should follow target
- `RobotHeadingDeg` — heading used in calculation, should match `Heading`

---

## 2. Why the 4th Wire and How It Helps

### What It Is

The Axon MAX servo has a built-in absolute encoder that outputs analog voltage
(0–3.3V) proportional to servo shaft position. This is the **4th wire** (blue/orange).

Only one wire needed — all 3 turret servos move identically, one encoder represents all.

### Connection

```
Axon MAX 4th wire → any Analog Input port (0-3) on Control Hub
Ground (black) → GND on same analog port
```

In hardware config: add `Analog Input` device named `"abs"`.
In code: set `ABS_ENABLED = true` in `ShooterMove`.

The calibration constants `m` and `b` need verification with `SERVO_CENTER=0.607`:
- Run TurretTest manual mode
- Compare `Abs Encoder (deg)` vs `Turret deg (calc)`
- Adjust `m` and `b` until they match at multiple positions

### What It Enables

**1. Actual position in candidate picker**
Currently `lastChosenDeg` = commanded position (may differ from actual if servo is mid-travel).
With abs encoder: use real position → candidate picker never picks the wrong wrap candidate.

**2. Velocity measurement**
```java
double actualVelocity = (actualPos - lastActualPos) / dt;  // turret °/s
// servo shaft velocity = actualVelocity × 1.408
```

**3. Safe direction reversal detection**
```java
double commandedDirection = Math.signum(chosen - lastChosenDeg);
double currentDirection = Math.signum(actualVelocity);
boolean reversal = stillMoving && (commandedDirection != currentDirection);
if (reversal && Math.abs(actualVelocity) > 50.0) {
    chosen = lastChosenDeg;  // hold — wait for deceleration
}
```

Servo shaft at 422°/s needs ~50-100ms to decelerate. Waiting until `actualVelocity < 50°/s`
(turret) = `<70°/s` servo shaft before reversing prevents overcurrent completely.

**4. Stall detection**
If `|commanded - actual| > 20°` for >5 loops → servo hub likely cut power.
Log warning, reset `lastChosenDeg` to actual position.

### Impact Summary

| Issue | Without 4th wire | With 4th wire |
|---|---|---|
| Direction reversal overcurrent | Possible at wrap | Prevented — wait for deceleration |
| Wrong candidate after hub cut | `lastChosenDeg` stale | Reset from actual position |
| Tracking accuracy during wrap | Commanded position only | Actual position tracked |
| Stall detection | None | Detected within 5 loops (~100ms) |

---

## 3. Final Software Limits for Driving

### Servo Range

| Parameter | Value | Notes |
|---|---|---|
| SERVO_CENTER | 0.607 | Confirmed: servo pos where turret faces forward |
| SERVO_MIN | 0.05 | Safe electrical limit (~20° from absolute end) |
| SERVO_MAX | 0.95 | Safe electrical limit (~20° from absolute end) |
| SERVO_RANGE_DEG | 355° | Axon MAX full travel |
| SERVO_TO_TURRET_RATIO | 1.408× | (48/15) × (47/107) |

### Turret Limits

| Parameter | Value | ServoPos | Margin from electrical |
|---|---|---|---|
| TURRET_MIN | -160° | 0.927 | ~6° from SERVO_MAX=0.95 |
| TURRET_MAX | +265° | 0.077 | ~6° from SERVO_MIN=0.05 |
| Total range | **425°** | 0.077–0.927 | Safe both sides |

### Driving Implications

The turret covers 425° — any robot heading is shootable without repositioning.
Wrap only triggers when `targetAngleDeg` crosses -160° or +265°.

`targetAngleDeg = atan2(goal, robot)° - robotHeading°`

#### Red Alliance — Goal at (138, 138), robot in back-right triangle zone

| Robot position | Robot heading | atan2 to goal | targetAngleDeg | Wrap? |
|---|---|---|---|---|
| (72, 72) apex | 90° | 45° | -45° | No |
| (72, 72) apex | 0° (facing right) | 45° | +45° | No |
| (72, 72) apex | 180° (facing left) | 45° | -135° | No |
| (72, 72) apex | 270° (facing away) | 45° | +135° | No |
| (108, 108) near goal | 90° | 45° | -45° | No |
| (108, 108) near goal | 180° | 45° | -135° | No |
| (24, 120) far diagonal | 90° | 128° | +38° | No |
| (24, 120) far diagonal | 0° | 128° | +128° | No |
| (24, 120) far diagonal | 270° | 128° | +218° | No — within +265° |
| Any position | full spin | varies | up to ±360° | **Possible** |

#### Blue Alliance — Goal at (6, 138), robot in back-left triangle zone

| Robot position | Robot heading | atan2 to goal | targetAngleDeg | Wrap? |
|---|---|---|---|---|
| (72, 72) apex | 90° | 135° | +45° | No |
| (72, 72) apex | 0° (facing right) | 135° | +135° | No |
| (72, 72) apex | 180° (facing left) | 135° | -45° | No |
| (72, 72) apex | 270° (facing away) | 135° | -135° | No |
| (36, 108) mid triangle | 90° | 135° | +45° | No |
| (36, 108) mid triangle | 0° | 135° | +135° | No |
| (12, 120) near goal | 90° | 108° | +18° | No |
| (12, 120) near goal | 270° (facing away) | 108° | -162° | **Yes — just past -160°** |
| (12, 120) near goal | 260° | 108° | -152° | No — within -160° |
| Any position | full spin | varies | up to ±360° | **Possible** |

**Key difference — Blue alliance:** The goal is at the back-LEFT corner (6, 138).
When the robot faces away (heading ~270°) near the goal, `targetAngleDeg` can reach
~-162° — just past TURRET_MIN=-160°. This is the tightest case for Blue alliance.

**Mitigation options for Blue alliance near-goal wrap edge case:**
- `LIMIT_WRAP_JUMP = true` handles it gracefully in ~60ms
- Alternatively raise `TURRET_MIN` to -170° (servoPos=0.947, still within SERVO_MAX=0.95)

**Conclusion:** Both alliances are safe during normal match driving.
Blue alliance has one edge case near the goal when facing directly away — handled by
`LIMIT_WRAP_JUMP`. Red alliance has no edge cases within the launch zone.

If `LIMIT_WRAP_JUMP = true`: wrap is handled gracefully in ~60ms with no hub beep.
If 4th wire connected: direction reversal prevented, tracking accuracy improved throughout.

---

### Driver Practice Guide — Minimizing Wrap

The turret's 425° range means **normal driving never triggers wrap**. However continuous
spinning in one direction will. Here is what triggers wrap and how to avoid it.

#### What Triggers Wrap

Wrap triggers when `robotHeading` crosses specific angles. These are fixed per alliance:

| Alliance | Goal | Wrap triggers at robotHeading |
|---|---|---|
| Red | (138, 138) | ~+205° or ~-220° from field zero |
| Blue | (6, 138) | ~+295° or ~-130° from field zero |

In plain terms: **wrap triggers when the robot has rotated more than ~200° from its
starting heading in one continuous direction.**

#### Driver Rules

**Rule 1 — Avoid continuous spinning.**
Spinning 180° to evade a defender is fine. Continuing to spin past 200° in the same
direction triggers wrap. If you need to turn more than 180°, reverse direction and
come back — the turret tracks correctly in both directions.

**Rule 2 — Blue alliance: avoid facing directly away from the goal near the goal.**
When close to the Blue goal (within ~42"), heading ~270° (facing away) pushes
`targetAngleDeg` to ~-162° — just past the limit. Either:
- Keep heading away from 270° when near the Blue goal, or
- Trust `LIMIT_WRAP_JUMP=true` to handle the 60ms gap

**Rule 3 — After a full defensive spin, re-center heading before shooting.**
If the robot was spun by a defender, rotate back toward the goal before pulling the
shoot trigger. The turret will have wrapped and needs ~60ms to recover with
`LIMIT_WRAP_JUMP=true`. A brief pause after re-centering ensures the turret has settled.

**Rule 4 — Watch the turret physically during practice.**
If the turret is visibly sweeping a large arc (>90°) before settling on the goal,
wrap triggered. Note what the robot heading was at that moment and practice avoiding
that heading during match play.

#### Practice Drill

1. Start at field center (72, 72), heading 90° (facing goal wall)
2. Drive naturally within the back triangle — shoot continuously
3. Gradually add more aggressive turns — 90°, 135°, 180° rotations
4. Note at what rotation angle the turret visibly sweeps before re-acquiring
5. That rotation angle is the wrap threshold — practice staying just inside it

#### Heading Reference (Pedro Pathing)

Pedro heading 0° = facing right (+X direction on field).
Goal wall is at Y=138 (top of field). Robot typically starts heading ~90°.

```
Red alliance typical heading range during match:  45° to 135°  (±45° from 90°)
Blue alliance typical heading range during match: 45° to 135°  (±45° from 90°)
Safe rotation budget before wrap:                ~±200° from starting heading
```

With normal match driving staying within ±90° of the goal-facing heading, **wrap
will not occur**. Only sustained defensive evasion requiring >200° continuous rotation
needs driver awareness.

### Recommended Configuration for Competition

```java
SERVO_CENTER      = 0.607   // confirmed physical center
TURRET_MIN        = -160.0  // 425° total range
TURRET_MAX        = +265.0
SERVO_MIN         = 0.05    // safe electrical margin
SERVO_MAX         = 0.95
LIMIT_WRAP_JUMP   = true    // prevent overcurrent during wrap
MAX_WRAP_JUMP_DEG = 90.0    // 3 loops × 20ms = 60ms wrap time
ENABLE_OMEGA_COMP = true    // angular velocity compensation (tune T_LAG first)
SERVO_T_LAG       = 0.04    // tune on Dashboard — expect 0.03–0.08
OMEGA_ALPHA       = 0.3     // EMA filter — lower = smoother, higher = more responsive
OMEGA_COMP_CLAMP  = 15.0    // safety limit on compensation degrees
ABS_ENABLED       = true    // once 4th wire is connected
```

---

## 4. Angular Velocity Compensation (Shoot-While-Moving)

### Problem

The Axon MAX servo has an internal PID that takes time to reach commanded positions.
When the robot rotates while shooting, the turret target changes each loop. But the
servo is always **behind** by its internal PID lag — typically 30–80ms. At high angular
velocity, this lag means the turret trails the goal by several degrees.

```
Example: Robot rotating at 200°/s, servo lag = 50ms
Tracking error = 200°/s × 0.050s = 10° behind the goal
At 94" distance, 10° turret error = ~16" miss at the goal
```

### Solution: Feedforward Compensation

Instead of commanding the turret to where the goal **is**, command it to where the goal
**will be** after the servo lag. This is a simple feedforward term:

```
compensation = -filteredOmegaDeg × SERVO_T_LAG
targetAngleDeg += compensation
```

The negative sign is because:
- Robot rotating clockwise → positive omega → goal moves **left** in robot frame
- To lead the turret, command it slightly **left** (negative direction in our convention)

### Implementation in ShooterMove

```java
// Feature-flagged — enable on Dashboard after basic tracking is confirmed
public static boolean ENABLE_OMEGA_COMP = false;
public static double SERVO_T_LAG = 0.04;      // seconds
public static double OMEGA_ALPHA = 0.3;        // EMA filter coefficient
public static double OMEGA_COMP_CLAMP = 15.0;  // max compensation degrees

// In periodic():
double rawOmegaRad = followerSupplier.get().getAngularVelocity();
double rawOmegaDeg = Math.toDegrees(rawOmegaRad);
filteredOmegaDeg = OMEGA_ALPHA * rawOmegaDeg + (1.0 - OMEGA_ALPHA) * filteredOmegaDeg;

if (ENABLE_OMEGA_COMP) {
    servoLagCompDeg = -filteredOmegaDeg * SERVO_T_LAG;
    servoLagCompDeg = clamp(servoLagCompDeg, -OMEGA_COMP_CLAMP, OMEGA_COMP_CLAMP);
    targetAngleDeg += servoLagCompDeg;
}
```

### Parameters

| Parameter | Default | Range | Purpose |
|---|---|---|---|
| `ENABLE_OMEGA_COMP` | false | — | Master switch — enable after basic tracking works |
| `SERVO_T_LAG` | 0.04 | 0.03–0.08 | Servo lag in seconds — the main tuning knob |
| `OMEGA_ALPHA` | 0.3 | 0.1–0.5 | EMA filter — lower = smoother but more delayed |
| `OMEGA_COMP_CLAMP` | 15.0 | 10–20 | Safety clamp — prevents runaway compensation |

### EMA Filter

The Pinpoint IMU angular velocity can be noisy. The exponential moving average (EMA)
filter smooths it:

```
filteredOmega = α × rawOmega + (1 - α) × filteredOmega
```

- `α = 0.3`: Moderate smoothing. Good default for 20ms loop time.
- `α = 0.1`: Very smooth but adds ~60ms delay — may negate the compensation benefit.
- `α = 0.5`: Responsive but noisy — can cause turret jitter when stationary.

### Tuning Procedure

1. **Confirm basic tracking first** — omega comp OFF, drive straight, verify turret tracks goal
2. Enable `ENABLE_OMEGA_COMP = true` on Dashboard
3. Drive **arcs** (constant rotation while moving forward) at ~50% speed
4. Watch Dashboard: `ServoLagCompDeg` should be 2–8° during arcs, ~0° when straight
5. Start with `SERVO_T_LAG = 0.04`
   - If shots **trail** the goal (behind rotation direction) → increase T_LAG
   - If shots **lead** the goal (ahead of rotation direction) → decrease T_LAG
   - If turret **jitters** when stationary → decrease OMEGA_ALPHA
6. Test at multiple speeds: slow arcs, fast arcs, spin-and-shoot
7. Final check: spin robot 180° quickly, shoot immediately — turret should acquire quickly

### Dashboard Telemetry

| Field | Description |
|---|---|
| `OmegaDegPerSec` | Filtered angular velocity — should be ~0 when stationary |
| `ServoLagCompDeg` | Compensation applied this loop — 0 when disabled |
| `TurretTargetDeg` | Target after compensation, before candidate picking |
| `TurretDeg` | Final commanded turret position |

### Interaction with Other Features

| Feature | Interaction |
|---|---|
| `LIMIT_WRAP_JUMP` | Compatible — omega comp adjusts target before candidate picking |
| 4th wire (abs encoder) | Independent — omega comp works without encoder |
| Shoot-while-moving solver | Complementary — solver handles ball flight time, omega comp handles servo lag |

### Future: Shoot-While-Moving Solver + Heading Prediction

The 10-iteration solver (currently commented out in ShooterMove) predicts where the
robot will be when the ball arrives at the goal. When re-enabled, it should also predict
the robot's **heading** at shot time:

```java
double headingForAngle = robotHeading + rawOmegaRad * shotTime;
```

This is separate from omega compensation:
- **Omega comp**: corrects for servo mechanical lag (~40ms)
- **Solver heading prediction**: corrects for ball flight time (~500ms)

Both should be active simultaneously for best accuracy at speed.

---

## 5. Testing & Deployment Sequence

Deploy features in this order. Each step must be confirmed working before enabling the next.

### Step 1: Verify Basic Tracking (Normalization Fix)

**What changed:** Removed `atan2(sin,cos)` normalization that created artificial ±180°
discontinuity. Target angle is now unbounded, matching V1 behavior.

**Test:**
1. Run `TeleopMoving`, filter logcat: `adb logcat -s Turret`
2. Drive forward, backward, left, right — turret should track goal smoothly
3. Watch Dashboard: `TurretTargetDeg` should change smoothly with heading
4. Rotate robot 360° slowly — turret should follow continuously
5. **Pass criteria:** No unexpected jumps in `TurretDeg`, no servo hub beep during normal driving

### Step 2: Test Wrap Jump Limiting

**Test with `LIMIT_WRAP_JUMP = false` first:**
1. Spin robot continuously in one direction past ~200° from start
2. Watch for `WRAP JUMP` in logcat and listen for servo hub beep
3. Note: beep is expected here — this confirms wrap occurs at turret limits

**Test with `LIMIT_WRAP_JUMP = true`:**
1. Same spin test — wrap should spread over ~3 loops (60ms)
2. No servo hub beep expected
3. Watch Dashboard: `TurretDeg` should ramp smoothly through the wrap
4. **Pass criteria:** Wrap completes without beep, turret re-acquires goal within ~100ms

### Step 3: Test Angular Velocity Compensation

**Prerequisites:** Steps 1-2 passing.

1. Set `ENABLE_OMEGA_COMP = true` on Dashboard
2. Drive straight — `ServoLagCompDeg` should be ~0°
3. Drive arcs — `ServoLagCompDeg` should be 2–8°
4. Shoot while driving arcs at 50% speed — compare accuracy with comp on vs off
5. Tune `SERVO_T_LAG` per the procedure in Section 4
6. **Pass criteria:** Shots land closer to center of goal during arcs with comp on

### Step 4: Wire 4th Wire (Absolute Encoder) — OPTIONAL

This step is **independent** — skip it if the hardware isn't ready. Steps 3 and 5
work without it. The encoder improves wrap recovery and enables stall detection,
but is not required for basic tracking, omega compensation, or the solver.
1. Connect one Axon MAX 4th wire to any Analog Input port (0-3) on Control Hub
2. Connect ground (black) to GND on same analog port
3. In hardware config: add `Analog Input` device named `"abs"`
4. Set `ABS_ENABLED = true` in ShooterMove (or on Dashboard)

**Calibration:**
1. Run `TurretTest` in MANUAL mode
2. Move turret to several known positions (0°, ±90°, limits)
3. Compare `Abs Encoder (deg)` vs `Turret deg (calc)`
4. Adjust `m` and `b` until they match at all positions
5. Copy confirmed `m`, `b` to both `TurretTest.java` and `ShooterMove.java`

**Verification:**
1. Run `TeleopMoving` — Dashboard should show abs encoder tracking commanded position
2. Drive and spin — abs encoder should follow turret with minimal delay
3. **Pass criteria:** Abs encoder reads within ±3° of commanded position at all angles

### Step 5: Re-enable Shoot-While-Moving Solver

**Prerequisites:** Steps 1-3 passing, LUT values validated for V2.
Step 4 (4th wire) is **optional** — the solver works with commanded position (`lastChosenDeg`).
The encoder improves accuracy during wraps but is not required.

1. Uncomment the 10-iteration solver in ShooterMove.periodic()
2. Add heading prediction: `headingForAngle = robotHeading + rawOmegaRad * shotTime`
3. Test at multiple distances: 42", 70", 91", 114", 131"
4. Compare shot accuracy: stationary vs moving at each distance
5. **Pass criteria:** Moving shots land within 1 ball-width of stationary shot grouping

---

## 6. Quick Reference — All Feature Flags

| Flag | Default | Purpose | When to Enable |
|---|---|---|---|
| `LIMIT_WRAP_JUMP` | false | Spread wrap jumps over multiple loops | Always on for competition |
| `MAX_WRAP_JUMP_DEG` | 90.0 | Degrees per loop during wrap | Tune if wrap feels slow |
| `ENABLE_OMEGA_COMP` | false | Angular velocity feedforward | After basic tracking confirmed |
| `SERVO_T_LAG` | 0.04 | Servo lag compensation (seconds) | Tune while driving arcs |
| `OMEGA_ALPHA` | 0.3 | Angular velocity EMA filter | Lower if turret jitters |
| `OMEGA_COMP_CLAMP` | 15.0 | Max compensation degrees | Raise if fast spins under-compensate |
| `ABS_ENABLED` | false | Read absolute encoder on 4th wire | After wiring + calibrating |
| `HOOD_ENABLED` | false | Control hood servo | After hood is installed |
| `ENABLE_FF` | false | Flywheel feedforward (kV, kS) | After kV/kS are characterized |
| `MANUAL_RPM` | 0 | Override flywheel target (rad/s) | For testing; 0 = use distance LUT |
| `MANUAL_POWER` | 0.5 | Raw flywheel power (bypasses PID) | Initial testing only; 0 = use PID |
