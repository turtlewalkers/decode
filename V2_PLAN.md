# V2 Robot Code Plan

## Status: IN DISCUSSION — implementation starting in phases

---

## Context

V1 code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
V2 is a new robot with mostly the same concepts but different hardware. The plan is to write new subsystems and opmodes for V2 while keeping V1 code intact. V2 work is on the `v3` branch. `Teleop5.java` is the current V2 test file.

---

## V1 Hardware Reference

| Name | Type | Role |
|------|------|------|
| `lf`, `rf`, `lb`, `rb` | DcMotorEx | Mecanum drivetrain |
| `intake` | MotorEx | Ball intake |
| `sb`, `st` | MotorEx | Shooter flywheel (bottom/top) |
| `turret` | MotorEx | Horizontal turret rotation |
| `hood` | ServoEx | Vertical shooter angle |
| `latch` | ServoEx | Ball gate |
| `abs` | AnalogInput | Absolute encoder for turret |
| `led` | RevBlinkinLedDriver | LED indicator |
| `limelight` | Limelight3A | AprilTag vision |
| `pinpoint` | GoBildaPinpointDriver | Odometry |
| `Control Hub` | VoltageSensor | Battery voltage compensation |

---

## V2 Hardware Changes

| Component | V1 | V2 |
|---|---|---|
| Intake | 1x MotorEx | 1x MotorEx (same) |
| Transfer | none | goBILDA 5000 Series 12VDC, 1:1 belt (50T:50T), ~5800 RPM |
| Turret | 1x Yellow Jacket 312 RPM MotorEx | 2x Axon MAX ServoEx (`turret_l1`, `turret_l2`) via REV Servo Hub @ 7.4V |
| Turret r1 | n/a | Axon MAX mechanically connected, servo **unplugged** (passive) |
| Abs encoder | AnalogInput on turret motor shaft | Same encoder, on r1 shaft (independent wire, still works) |
| Hood | ServoEx | Unchanged |
| Flywheel | sb/st MotorEx | Unchanged (retune LUTs) |
| Drivetrain | Mecanum + Pinpoint | Unchanged |
| Limelight | Limelight3A | Unchanged, optional |
| Memory | Static class | Unchanged |
| Constants | PedroPathing config | **V2 needs separate Constants.java** (different robot mass/tuning) |

---

## V2 Intake/Transfer Subsystem Design

### Hardware
- `intake` motor — same as V1 (continuous ball collection)
- `transfer` motor — goBILDA 5000 Series 12VDC motor
  - Belt drive to transfer roller: **50T:50T = 1:1 ratio**
  - Roller speed: ~5800 RPM (fast ball movement) ✓
  - Torque: 1.47kg·cm (low — stalls cleanly against latch) ✓
  - **No encoder** — use current-based stall detection
  - Normal current: ~0.25A, Stall current: ~9.2A (wide gap, reliable detection) ✓
- `latch` ServoEx — same as V1

### Control Flow

#### Left Trigger held (collect mode)
- Intake: **run**
- Transfer: **run** → stall detected → **stop transfer** (ball held at closed latch)
- Latch: **closed**

#### Right Trigger held (shoot mode)
- Latch: **open** (stays open entire time trigger is held)
- Intake: **run**
- Transfer: **run** continuously (no stall detection — latch is open, balls flow freely)
- Release RIGHT → latch **close**, intake **stop**, transfer **stop**

#### Neither trigger
- Intake: **stop**
- Transfer: **stop**
- Latch: **closed**

### Transfer Stall Detection (collect mode only)
- Method: **current-based** (no encoder on 5000 Series motor)
- `transferMotor.getCurrent(CurrentUnit.AMPS) > STALL_CURRENT` for N consecutive loops → stop transfer
- Tunable via `@Config` + FTC Dashboard:
  ```java
  public static double STALL_CURRENT = 3.0;  // amps (between 0.25A normal and 9.2A stall)
  public static int STALL_LOOPS = 2;          // ~40ms — stop fast to protect motor
  ```
- Startup false-positive protection: ignore stall detection for first N loops after transfer starts
- No stall detection in shoot mode (latch open, nothing to stall against)

### Transfer State Machine
```
LEFT HELD:
  STOPPED → (trigger pressed) → RUNNING
  RUNNING → (current > STALL_CURRENT, STALL_LOOPS) → STOPPED (ball at latch)

RIGHT HELD:
  RUNNING continuously (no stall logic)
  STOPPED → (trigger released) → latch close, all stop

NEITHER:
  everything stopped
```

---

## V2 Subsystem Plan

### New / Changed Subsystems
- `v2/subsystems/Intake.java` — intake + transfer motor + current stall detection
- `v2/subsystems/ShooterMove.java` — servo turret (l1, l2) + flywheel + hood

### Unchanged / Carried Over
- `Memory.java` — unchanged
- `subsystems/Limelight.java` — unchanged, optional use in V2
- `ShooterMove` flywheel — same `sb`/`st` dual MotorEx, same PID+FF structure
  - **All values need retuning on V2 robot**: `p, i, d`, `kV, kS`, RPM LUT, hood angle LUT, shot time LUT
  - Tunable via `@Config` + FTC Dashboard
- Shoot-while-moving solver (10-iteration shot-time) — carries over unchanged, only final turret output line changes

---

## V2 Turret Design

### Hardware
- 3x **Axon MAX ServoEx** replacing the V1 MotorEx
  - `turret_l1`, `turret_l2` — left side, same direction
  - `turret_r1` — right side, reversed (opposite side of gear)
  - All 3 connected via **REV Servo Hub** (7.4V, 15A total, synchronized commands)
- Power budget: 3 × ~3.5A stall = ~10.5A peak, within 15A hub limit ✓
- `abs` AnalogInput — same encoder, same calibration (`m = -123.71, b = 256.37`)
  - Mounted on servo shaft (before gearing)
  - Reads 0-355° (no wrap, no fusion needed)
- `hood` ServoEx — unchanged from V1

### Gear Ratios
- Stage 1: 48t:15t = 3.2x
- Stage 2: 47t:107t = 0.4393x
- Combined: 3.2 × 0.4393 = **1.408x** (used in code)
- Full servo range 355° × 1.408 = **~500° turret output**

### PWM Range
- FTC default PWM: 750–2250µs → only **~167°** of Axon MAX travel used (insufficient)
- Axon MAX full range requires: **500–2500µs**
- Set in code: `((PwmControl) hMap.get(Servo.class, "tl1")).setPwmRange(new PwmControl.PwmRange(500, 2500))`
- Applied to all 3 servos (tl1, tl2, tr1) in both TurretTest and ShooterMove constructors

### Servo Position Limits
- FTC servo position range: 0.0 → 1.0 (maps to full PWM range after setPwmRange)
- **Raw limits** 0.0 and 1.0 map to physical endpoints — risk of hitting mechanical stops at full speed
- **Safe limits**: `SERVO_MIN = 0.03`, `SERVO_MAX = 0.97` — leaves ~30° margin each side
- Calculation:
  ```
  Full servo range with 500-2500µs:        355°
  Usable range (0.03 → 0.97):              0.94 × 355° = 333.7° servo shaft
  Turret output range:                     333.7° × 1.408 = ~470° turret
  Soft limit span (TURRET_MIN to MAX):     -130° to 255° = 385°
  Margin vs usable turret range:           470° - 385° = ~85° total (~42° each side)
  ```
- Why not 0.0/1.0: Axon MAX at full travel can stall against mechanical stops, risking gear/servo damage
- Why not 0.02/0.98: Chosen 0.03/0.97 for slightly more margin — confirmed acceptable range

### Speed & Torque
- Axon MAX native speed: ~125°/sec @ 6V, ~167°/sec @ 7.4V
- Turret output speed: ~89°/sec @ 6V, ~119°/sec @ 7.4V (comparable to V1's ~130°/sec)
- Turret output torque: 3 × 25kg·cm × 1.408 ≈ **105kg·cm** (ensure mechanical stops are robust)
- Recommend setting REV Servo Hub to **7.4V** for best speed

### Position Calculation
```
absVoltageDeg = abs.getVoltage() * m + b                     // servo shaft degrees (0–355°)
turretPosDeg  = absVoltageDeg * (48.0/15.0) * (47.0/107.0)  // actual turret degrees (~0–500°)
```

### Control Architecture
- Servos are in **standard position mode** — no PID needed, servo handles position control internally
- `SERVO_CENTER` is the servo position [0,1] where turret is at 0° (facing forward) — find via TurretTest
- Coordinate conversion:
  ```
  // turret degrees → servo position
  pos = SERVO_CENTER + targetTurretDeg / (355.0 × 1.408)
  pos = clamp(pos, 0.0, 1.0)

  // servo position → turret degrees (for fused position)
  turretDeg = (pos - SERVO_CENTER) × 355.0 × 1.408
  ```
- Servo commands — all 3 same direction (confirmed on robot):
  ```
  l1.set(pos)
  l2.set(pos)   // same direction — both on same face of 48t gear
  r1.set(pos)   // same direction — confirmed R1_MIRRORED = false, flag removed
  ```
- Servo position clamped to `[SERVO_MIN, SERVO_MAX]` = `[0.03, 0.97]` in `turretDegToServoPos()`
- `TURRET_MIN`, `TURRET_MAX` in turret degrees — TBD from mechanical stops via TurretTest

### Slew Rate Limiting
- **Why**: Servo position mode is instantaneous — without a step limit, a large target jump would
  slam the turret into the mechanical stops at full speed, risking gear damage.
  V1 didn't need this because the motor PID naturally ramped acceleration.
- **How**: Each loop, cap the servo position change to `MAX_SERVO_STEP`.
  Near mechanical limits (within `EDGE_ZONE_DEG`), linearly reduce to `MIN_SERVO_STEP`.
- All tunable via `@Config` + FTC Dashboard: `MAX_SERVO_STEP`, `MIN_SERVO_STEP`, `EDGE_ZONE_DEG`

### Change Detection
- Skip writing to servo hub if position hasn't changed since last loop
- **Why**: Avoids hammering the REV Servo Hub with redundant commands every loop

### Fused Position (simplified from V1)
- V1 fused relative encoder + abs encoder via complementary filter (needed because absolute encoder
  had limited range and relative encoder drifted)
- V2: abs encoder only — no motor encoder on servo turret
- Complementary filter still runs but purely corrects against abs encoder each loop:
  ```
  k = 1 - exp(-dt / TURRET_TAU)
  fusedPos += k × angleDiff(absEncoderDeg, fusedPos)
  ```
- **Why keep the filter**: abs encoder can have momentary noise spikes; filter smooths these out
  rather than reacting instantly to every reading

### Shoot-While-Moving (carried over from V1 unchanged)
- Same 10-iteration solver from V1 `ShooterMove.periodic()`
- Only change from V1: final output line replaced from `turret.set(motorPower)` → `setTurretDeg(chosen)`
- All field geometry, velocity compensation, and candidate selection logic identical

### Simplified vs V1
| | V1 | V2 |
|---|---|---|
| Turret actuator | 1× MotorEx + ProfiledPID | 3× ServoEx position mode |
| Position sensing | Relative encoder + abs encoder (fused) | Abs encoder only (fused for noise) |
| Acceleration control | ProfiledPID trapezoidal profile | Slew rate limiting per loop |
| Wrap-around handling | ±360° candidate selection | Same — carried over from V1 |
| PID tuning needed | Yes (pT, iT, dT, maxV, maxA) | No — servo handles internally |

### Final Turret Decisions
- **All 3 servos wired**: `l1`, `l2`, `r1` — sufficient torque and speed confirmed
  - l1 + l2: same face of 48t gear → same direction
  - r1: opposite side of turret gear → **confirmed `R1_MIRRORED = false`** (same direction as l1/l2)
- **Absolute encoder on r1 shaft** — independent wire, reads turret position regardless of servo signal ✓

### Mechanical Layout (confirmed)
```
l1 → 48t servo gear ──┐
                        ├──→ 15t idler ──→ 47t idler ──→ 107t turret gear ←── r1 (wired, mirrored)
l2 → 48t servo gear ──┘                                         ↑
                                                          abs encoder (independent wire)
```

### Speed & Torque (confirmed)
- Combined ratio: **1.408x**
- Turret range: 355° × 1.408 = **~500°** ✓
- Turret speed: ~125°/sec × 1.408 = **~176°/sec @ 6V** (faster than V1's ~130°/sec) ✓
- Turret torque: 2 × 25kg·cm × 1.408 = **~70kg·cm** (~3× V1's 24.3kg·cm) ✓

### Shooting Distance
- Minimum shooting distance is **hood-limited (~42in)**, not turret-speed-limited
- At 42in worst case perpendicular movement: ~68°/sec required, V2 handles easily ✓
- Turret tracking fine for all realistic competition scenarios

---

## V2 Code Structure

```
teamcode/
  pedroPathing/
    Constants.java           ← SHARED V1 (unchanged)
  robot/
    Memory.java              ← SHARED (unchanged)
  subsystems/                ← V1 subsystems (unchanged)
    Intake.java
    ShooterMove.java
    Shooter.java
    Limelight.java
  teleop/
    TeleopMoving.java        ← V1 (unchanged)
    Teleop5.java             ← V2 current test file
  v2/
    pedroPathing/
      Constants.java         ← V2 PedroPathing tuning (separate from V1)
    subsystems/
      Intake.java            ← V2 (intake + transfer + stall detection)
      ShooterMove.java       ← V2 (servo turret)
    teleop/
      TeleopMoving.java      ← V2 final teleop (@TeleOp group="V2")
```

### Opmode Grouping on Driver Hub
```java
@TeleOp(name = "TeleopMoving", group = "V1")  // V1
@TeleOp(name = "TeleopMoving", group = "V2")  // V2
```

---

## Phased Implementation Plan

### Phase 1: Intake + Transfer + Latch (DONE — code complete)
**Goal**: Test intake/transfer/latch sequences on V2 hardware

- [x] Create `v2/subsystems/Intake.java` with transfer motor + current stall detection
- [x] Wire left trigger → intake + transfer (collect mode, stall stops transfer)
- [x] Wire right trigger → latch open + intake + transfer run continuously (shoot mode)
- [x] Wire neither trigger → all stop, latch close
- [x] Create `v2/teleop/HardwareTest.java` — individual motor/servo health check with stall protection
- [x] Test and tune `STALL_CURRENT` threshold — confirmed 3.0A
- [ ] Hardware map names: `intake`, `transfer`, `latch`

### Phase 2: Drivetrain + Odometry
**Goal**: Verify PedroPathing works on V2 robot

- [ ] Create `v2/pedroPathing/Constants.java` with V2-specific tuning
- [ ] Run PedroPathing tuning routines on V2 robot
- [ ] Verify driving in `Teleop5.java` works correctly

### Phase 3: Turret
**Goal**: Test servo turret auto-aim

- [x] Create `v2/subsystems/ShooterMove.java` with servo turret (l1, l2, r1 all wired)
- [x] Create `v2/shooter/TurretTest.java` — abs encoder verification, SERVO_CENTER, R1_MIRRORED, mechanical stop limits
- [x] `ABS_ENABLED` flag added — abs encoder optional, hardware only initialized when enabled
- [x] `HOOD_ENABLED` flag added — hood optional, hardware only initialized when enabled
- [ ] Run TurretTest: verify abs encoder calibration (`m`, `b`) reads ~0° at center
- [ ] Run TurretTest: find and set `SERVO_CENTER`
- [ ] Run TurretTest: verify l1/l2 direction (disconnect r1 first)
- [x] Run TurretTest: confirm `R1_MIRRORED` value — confirmed `false` (same direction as l1/l2)
- [ ] Run TurretTest: drive to mechanical stops, record → set `TURRET_MIN`, `TURRET_MAX`
- [ ] Hardware map names: `tl1`, `tl2`, `tr1`, `abs`

### Phase 4: Flywheel + Hood
**Goal**: Test shooter with new turret

- [x] Create `v2/shooter/Shooter.java` — flywheel PID tuning opmode (uses rad/s throughout, matching V1; p/i/d copy directly to ShooterMove)
- [x] Create `v2/shooter/ShooterTest.java` — FF characterization opmode (kV, kS)
- [x] `MANUAL_POWER` override added to ShooterMove — bypasses PID for raw power testing
- [x] `MANUAL_RPM` override added to ShooterMove — overrides LUT when > 0
- [x] Hood connected as optional (`HOOD_ENABLED = true/false` via Dashboard)
- [ ] Run ShooterTest: sweep power, fit kV and kS for V2
- [ ] Run Shooter.java to tune p/i/d:

  **Step 1 — Baseline FF (ShooterTest opmode, no balls)**
  - Sweep power, fit `kV` and `kS`
  - Gets flywheel to target RPM open-loop before PID is involved

  **Step 2 — Tune PID alone, no balls (Shooter opmode)**
  - Set `ENABLE_FF = false`, `f = 0.0025`, `TARGET_RAD = 300` (rad/s ≈ 2865 RPM, ~0.55 power)
  - Motor range confirmed: ~130 rad/s @ 0.3 power, ~543 rad/s @ full power
  - Useful tuning range: 200–450 rad/s
  - Note: `TARGET_RAD` is in rad/s — same units as V1, so p/i/d values copy directly from V1
  - Increase `p` until flywheel reaches target cleanly without oscillation
  - Add small `i` to kill steady-state error
  - Goal: PID + simple FF holds target stably before kV/kS are added

  **Step 3 — Enable FF, re-verify at rest**
  - Set `ENABLE_FF = true` with kV/kS from Step 1
  - FF now carries the base load — PID error will be much smaller
  - You may need to reduce `p` slightly since FF already gets close to target
  - Goal: flywheel reaches target faster and holds it with less PID effort

  **Step 4 — 1 ball, right trigger only**
  - Load 1 ball manually into latch (no intake)
  - Press right trigger, watch RPM dip on Dashboard
  - Tune `p` until recovery is fast after single dip
  - Baseline: how much does 1 ball cost in RPM?

  **Step 5 — 1 ball, left trigger → right trigger**
  - Hold left trigger until transfer stalls (1 ball loaded)
  - Press right trigger to shoot
  - Should match Step 4 — confirms stall detection is working correctly

  **Step 6 — 3 balls, left trigger → right trigger**
  - Hold left trigger until transfer stalls (all 3 loaded)
  - Press right trigger, watch 3 consecutive dips on Dashboard
  - Each dip must recover before next ball arrives
  - If recovery too slow: increase `p` or reduce `TRANSFER_SPEED`
  - If overshoot between dips: `p` too high, reduce slightly

  **Step 7 — Verify 3-ball consistency**
  - Repeat Step 6 multiple times, check consistency across runs
  - Fine-tune `p` and `kV` if dip depth or recovery varies between runs

  **Step 8 — Copy to ShooterMove**
  - Copy confirmed `p`, `i`, `d`, `kV`, `kS`, `ENABLE_FF` to `ShooterMove.java`
  - Set `MANUAL_POWER = 0` to switch from raw power mode to PID mode

  **Key Dashboard metrics:**

  | What you see | What to do |
  |---|---|
  | RPM doesn't reach target | Increase `p` or `f` |
  | RPM oscillates at rest | Decrease `p` |
  | Dip too deep per ball | Increase `p` or `kV` |
  | Doesn't recover before next ball | Decrease `TRANSFER_SPEED` |
  | Overshoots after each dip | Decrease `p` |
  | Steady-state error below target | Increase `i` (small steps) |

- [ ] Retune RPM LUT, hood angle LUT, shot time LUT on V2 robot
- [ ] Connect hood and tune when ready
- [ ] Test shoot-while-moving

### Phase 5: Full Teleop
**Goal**: Complete V2 teleop

- [x] Create `v2/teleop/TeleopMoving.java` combining all subsystems
- [x] Wire all gamepad bindings (see Gamepad Bindings section below)
- [x] Gate safety system carried over from V1 (reaction time + decel model)
- [ ] Tune park/stay pose coordinates on V2 robot
- [ ] Add Limelight bindings when ready (LB/RB currently free)

### Phase 6: Autonomous
**Goal**: V2 autonomous routines

#### Code change analysis (from V1 Blue18/Red18)
Structure is 95% reusable — paths, command sequencing, Memory handoff, PedroPathing follower
are all identical. Only changes needed per file:

| What | Change | Effort |
|---|---|---|
| Subsystem imports | `subsystems.Intake` → `v2.subsystems.Intake`, `subsystems.Shooter` → `v2.subsystems.ShooterMove` | 2 lines |
| PedroPathing import | `pedroPathing.Constants` → `v2.pedroPathing.Constants` | 1 line |
| Constructor | `new Shooter(hMap, follower, x, y, true)` → `new ShooterMove(hMap, follower, x, y)` | 1 line |
| Intake constructor | `new Intake(hMap, follower, x, y)` → `new Intake(hMap)` (no follower needed in V2) | 1 line |
| Object rename | `shooter.flywheel/turretOff` → `shooterMove.flywheel/turretOff` | search & replace |
| Intake command names | see mapping table below | search & replace |
| Pose coordinates | field positions — same field, same game | retune on robot |
| WaitCommand timings | shooter settling times | retune on robot |

#### Intake command mapping (V1 → V2)
| V1 | V2 | Notes |
|---|---|---|
| `intake.open()` | `intake.shootStart()` | opens latch + runs intake + transfer |
| `intake.close()` | `intake.shootStop()` | closes latch + stops intake + transfer |
| `intake.collect()` | `intake.collectStart()` | runs intake + transfer, stall stops transfer |
| `intake.stop()` | `intake.collectStop()` | stops intake + transfer, closes latch |

#### Tasks
- [ ] Create `v2/pedroPathing/Constants.java` (Phase 2 dependency — must be done first)
- [ ] Copy `Blue18.java` → `v2/autonomous/Blue18.java`, apply import + constructor + rename changes
- [ ] Copy `Red18.java` → `v2/autonomous/Red18.java`, apply import + constructor + rename changes
- [ ] Retune `Pose` coordinates on V2 robot
- [ ] Retune `WaitCommand` timings on V2 robot
- [ ] Verify `Memory` pose handoff works correctly auto→teleop

---

## V2 Teleop Gamepad Bindings (`v2/teleop/TeleopMoving.java`)

### Gamepad 1 (Driver)

| Input | Action |
|-------|--------|
| Left stick | Drive (field-centric) |
| Right stick X | Rotate |
| Left trigger | Collect — intake + transfer run; stall stops transfer (ball held at latch) |
| Right trigger | Shoot — latch open, intake + transfer run continuously |
| DPAD down | Reverse intake |
| DPAD up | Hold position (Stay path) |
| A | Flywheel on |
| B | Flywheel off |
| Y | Park (drive to end pose) |
| X | Cancel auto command, resume manual drive |

### Gamepad 2 (Operator / Offset Tuning)

| Input | Action |
|-------|--------|
| DPAD up | Hood angle offset +0.05 |
| DPAD down | Hood angle offset -0.05 |
| Left bumper | Turret angle offset +5° |
| Right bumper | Turret angle offset -5° |
| A | Zero all offsets (hood + turret) |
| B | Manual relocalize (snap pose to known field position) |
| DPAD right | Toggle alliance (RED ↔ BLUE) |

### Notes
- Flywheel starts **on** at init (`MANUAL_POWER = 0.5` default during testing, set to 0 for PID mode)
- `MANUAL_POWER > 0` → raw power mode (bypasses PID, for initial testing)
- `MANUAL_POWER = 0, MANUAL_RPM > 0` → fixed RPM target via PID
- `MANUAL_POWER = 0, MANUAL_RPM = 0` → distance-based RPM from LUT (competition mode)
- LB/RB on gamepad1 reserved for Limelight when connected
- Gate safety active at all times — prevents driving through the gate opening

---

## V2 Teleop5 Test Bindings (`teleop/Teleop5.java`)

Simplified test opmode for hardware verification:

| Input | Action |
|-------|--------|
| Left trigger | Collect |
| Right trigger | Shoot |
| DPAD down | Reverse intake |
| Y | Toggle flywheel on/off |
| X | Cancel auto, resume manual |

---

## Open Questions / TODO (confirm from CAD)

- [ ] Transfer belt pulley tooth counts — assumed 50T:50T (1:1), confirm
- [ ] Transfer motor exact model — assumed goBILDA 5000 Series 12VDC, confirm
- [ ] Intake motor model/specs
- [ ] Transfer roller diameter (informational)
- [x] Latch servo — **confirmed**: arc-shaped servo arm, reversed geometry (`LATCH_OPEN = 0.71`, `LATCH_CLOSED = 0.95`)
- [x] Stall detection threshold — confirmed `STALL_CURRENT = 3.0A`
- [ ] Turret soft limits (`TURRET_MIN`, `TURRET_MAX`) in degrees — from mechanical stops via TurretTest
- [x] r1 wired and direction confirmed — `R1_MIRRORED = false` (same direction as l1/l2), flag removed
- [ ] Limelight camera position change on V2?
- [x] Abs encoder optional — `ABS_ENABLED` flag, hardware only initialized when true
- [x] Hood optional — `HOOD_ENABLED` flag, hardware only initialized when true
