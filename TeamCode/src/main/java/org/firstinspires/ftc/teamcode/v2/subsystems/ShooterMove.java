package org.firstinspires.ftc.teamcode.v2.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.Memory;

import java.util.function.Supplier;

@Config
public class ShooterMove extends SubsystemBase {

    // --- Absolute encoder calibration (same as V1 — confirm on V2 robot) ---
    public static double m = -123.71, b = 256.37;

    // --- Turret angle limits in turret degrees (TBD from mechanical stops, max ~500°) ---
    public static double TURRET_MIN = -160.0; //-140.0;
    public static double TURRET_MAX = 265.0; //245.0;

    // --- Turret servo geometry ---
    // Axon MAX: 355° range. Gear ratio: 48t:15t × 47t:107t = 1.408x
    public static double SERVO_RANGE_DEG = 355.0;
    public static double SERVO_TO_TURRET_RATIO = (48.0 / 15.0) * (47.0 / 107.0); // ~1.408
    public static double SERVO_CENTER = 0.59;//0.607; // tune on real robot — center of turret travel
    // Safe servo limits — 0.03/0.97 avoids physical endpoints (~30° margin each side)
    // Usable servo range: 333.7° → turret range: ~470° through 1.408x gear ratio
    public static double SERVO_MIN = 0.03;//0.1;//0.03;
    public static double SERVO_MAX = 0.97;//0.97;

    public static double SERVO_OFFSET_TL1 = -0.0028; //
    public static double SERVO_OFFSET_TL2 = 0.0025;
    public static double SERVO_OFFSET_TR1 = -0.0003;

    // --- Abs encoder ---
    public static boolean ABS_ENABLED = false;

    // --- Hood ---
    public static boolean HOOD_ENABLED = true;

    // --- r1 direction: all three servos confirmed same direction ---

    // --- Slew rate limiting ---
    public static boolean DIRECT_WRITE = true;  // true = write target directly (use with Axon Power Limit 75%), false = slew-limit every step
    public static boolean EDGE_ZONE_ENABLED = true;  // true = slow down near TURRET_MIN/MAX, false = trust servo + Axon Power Limit
    public static double MAX_SERVO_STEP = 0.03;  // max position change per loop (only used near edges when DIRECT_WRITE=true)
    public static double MIN_SERVO_STEP = 0.003;  // min step near limits
    public static double EDGE_ZONE_DEG  = 20.0;  // degrees from limit where slew starts reducing

    // --- Angular velocity compensation ---
    // Feedforward: leads the turret command by omega * T_LAG to compensate for servo lag
    // during robot rotation. Only active when ENABLE_OMEGA_COMP = true.
    // Tune T_LAG on Dashboard while driving arcs: increase until shots stop trailing the goal.
    public static boolean ENABLE_OMEGA_COMP = false;
    public static double SERVO_T_LAG = 0.04;     // seconds (expect 0.03–0.08)
    public static double OMEGA_ALPHA = 0.3;       // EMA filter coefficient (0–1, lower = smoother)
    public static double OMEGA_COMP_CLAMP = 15.0; // max compensation degrees (safety clamp)

    // Published for telemetry
    public static double omegaDegPerSec = 0;
    public static double servoLagCompDeg = 0;

    // --- Flywheel PID + FF (carry over from V1 — retune on V2) ---
    public static double p = 0.62, i = 0.02, d = 0.0035;
    public static boolean ENABLE_FF = true;
    public static double kV = 0.02049082; //0.022312028;//0.021477551;
    public static double kS = 0.499555731; //0.323009673;//0.760983135;
    public static double f = 0.0025;
    // Set > 0 to override LUT and run at a fixed target (rad/s, bypasses distance LUT)
    public static double MANUAL_RPM = 0;
    // Set 0.0–1.0 to drive flywheel at raw power (bypasses PID — use for initial testing)
    public static double MANUAL_POWER = 0;

    // --- Turret offset adjustment ---
    public static double turretOffset = 0;

    // --- Turret position (published for telemetry / other subsystems) ---
    public static double turretPosDeg = 0;

    public static double turretTargetDeg = 0;
    public static double robotHeadingDeg = 0;

    // --- Shooter position on field (set by teleop, same as V1) ---
    public static double TURRET_FWD_OFFSET = -0.3937;//-1.63;
    public static double TURRET_LEFT_OFFSET = 0.0;

    public static double batteryVoltage = 0.0;

    // Published for Intake to read during shoot mode
    public static double transferSpeedTarget = 0.6;
    private double turretOff = 1;

    // Hardware
    private final ServoEx turretL1, turretL2, turretR1;
    private final Servo hood;  // null when HOOD_ENABLED = false
    private final MotorEx shooterB, shooterT;
    private final AnalogInput abs;  // null when ABS_ENABLED = false
    private final VoltageSensor volt;
    private final Supplier<Follower> followerSupplier;

    private double lastChosenDeg = 0.0;

    // State
    private boolean flywheelOn = true;

    private boolean SWM = true;
    private double hoodOffset = 0;
    private final PIDController controllerShooter;
    private double shooterX, shooterY;

    private double filteredOmegaDeg = 0.0;
    private double currentServoPos;
    private boolean hasAppliedServoPos = false;
    private double lastAppliedServoPos = SERVO_CENTER;

    // Fused turret position (abs encoder only — no motor encoder in V2)
    private double fusedTurretPos = 0.0;
    private ElapsedTime timer;
    private static final double TURRET_TAU = 0.1;
    private static final double CORRECTION_DEADBAND = 0.05;

    private ElapsedTime loopTimer = new ElapsedTime();
    public static double loopTimeMs = 0;
    private Double fixedAngleDeg = null;

    // LUTs (carry over from V1 — retune on V2)
    InterpLUT RPM      = new InterpLUT();
    InterpLUT angle    = new InterpLUT();

    InterpLUT transfer    = new InterpLUT();
    InterpLUT shottime = new InterpLUT();

    public ShooterMove(HardwareMap hMap, Supplier<Follower> followerSupplier,
                       double shooterX, double shooterY) {
        this.followerSupplier = followerSupplier;
        this.shooterX = shooterX;
        this.shooterY = shooterY;

        turretL1 = new ServoEx(hMap, "tl1");
        turretL2 = new ServoEx(hMap, "tl2");
        turretR1 = new ServoEx(hMap, "tr1");

        // Axon MAX requires 500–2500µs PWM range (FTC default is 750–2250µs).
        // Without this, the servo only uses part of its travel and won't reach true 0° or 355°.
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);
        ((PwmControl) hMap.get(Servo.class, "tl1")).setPwmRange(axonRange);
        ((PwmControl) hMap.get(Servo.class, "tl2")).setPwmRange(axonRange);
        ((PwmControl) hMap.get(Servo.class, "tr1")).setPwmRange(axonRange);
        // hood     = HOOD_ENABLED ?  hMap.get(Servo.class, "hood") : null;
        shooterB = new MotorEx(hMap, "sb");
        shooterT = new MotorEx(hMap, "st");
        abs      = ABS_ENABLED ? hMap.get(AnalogInput.class, "abs") : null;
        volt     = hMap.get(VoltageSensor.class, "Control Hub");

        shooterB.setRunMode(MotorEx.RunMode.RawPower);
        shooterT.setRunMode(MotorEx.RunMode.RawPower);
        shooterB.setInverted(true);
        hood = hMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);

        // Initialize from actual servo position — Axon MAX is absolute, always knows where it is.
        // This avoids the 0° assumption when the turret is physically at a different angle at startup.
        currentServoPos = turretL1.getRawPosition();
        fusedTurretPos = servoPosToTurretDeg(currentServoPos);

        timer = new ElapsedTime();
        timer.reset();

        controllerShooter = new PIDController(p, i, d);
        turretOff(false);

        /* Lookup table values from 11:15 pm April 05

        distance, rad, hood, transferspeed
        29, 260, 1, 0.55
        35.5, 262, 0.7, 0.7
        43, 270, 0.75, 1
        49.5, 280, 0.6, 1
        56.5, 285. 0.45, 1
        68, 300, 0.35, 1
        76, 310, 0.3, 0.9
        83, 325, 0.23, 0.9
        91, 335, 0.2, 0.9
        103, 355, 0.18, 0.9
        114, 370, 0.12, 0.9
        130, 400, 0.10, 0.8
        142, 435, 0.08, 0.6
         */
                /* Lookup table values from 6:00pM April 18

        distance, rpm, hood, transferspeed, shottime
        29, 260, 1, 0.55, 0.6

        35.5, 250, 0.6, 0.7, 0.65

        43, 255, 0.7, 1, 0.5

        49.5, 257, 0.7, 1, 0.55

        56.5, 263. 0.45, 1, 0.54

        68, 272, 0.3, 1, 0.6

        76, 287, 0.25, 1, 0.64

        83, 304, 0.2, 1, 0.7

        91, 310, 0.19, 1, 0.7

        103, 330, 0.17, 0.9, 0.72

        114, 348, 0.2, 0.75, 0.76

        130, 370, 0.20, 0.75, 0.87

        142, 410, 0.3, 0.77, 1
         */
        /*
        distance, rad, hood, transferspeed
        29, 260, 1, 0.55
        35.5, 262, 0.7, 0.7
        43, 270, 0.75, 1
        49.5, 280, 0.6, 1
        56.5, 285. 0.45, 1
        68, 300, 0.35, 1
        76, 310, 0.3, 0.9
        83, 325, 0.23, 0.9
        91, 335, 0.2, 0.9
        103, 355, 0.18, 0.9
        114, 370, 0.12, 0.9
        130, 400, 0.10, 0.8
        142, 435, 0.08, 0.6
                */

                 /*
        distance, rad, hood, transferspeed
        29, 260, 1, 0.55
        35.5, 262, 0.7, 0.7
        43, 270, 0.75, 1
        49.5, 280, 0.6, 1
        56.5, 285. 0.45, 1
        68, 300, 0.35, 1
        76, 310, 0.3, 0.9
        83, 325, 0.23, 0.9
        91, 335, 0.2, 0.9
        103, 355, 0.18, 0.9
        114, 370, 0.12, 0.9
        130, 400, 0.10, 0.8
        142, 435, 0.08, 0.6
                */
        // LUTs — V1 values, retune on V2
        RPM.add(0, 225);
        RPM.add(27, 225);
        RPM.add(43, 237);
        RPM.add(59, 252);
        RPM.add(67, 265);
        RPM.add(75, 275);
        RPM.add(83, 296);
        RPM.add(91, 304);
        RPM.add(99, 320);
        RPM.add(107, 335);
        RPM.add(118, 365);
        RPM.add(131, 375);
        RPM.add(147, 395);
        RPM.add(3000, 435);
        RPM.createLUT();

        angle.add(0, 0.9313);
        angle.add(27, 0.97);
        angle.add(43, 0.63);
        angle.add(59, 0.66);
        angle.add(67, 0.50);
        angle.add(75, 0.365);
        angle.add(83, 0.21);
        angle.add(91, 0.205);
        angle.add(99, 0.20);
        angle.add(107, 0.18);
        angle.add(118, 0.19);
        angle.add(131, 0.13);
        angle.add(147, 0.08);
        angle.add(3000, 0.09);
        angle.createLUT();

        transfer.add(0, 1);
        transfer.add(43, 1);
        transfer.add(59, 0.925);
        transfer.add(67, 0.91);
        transfer.add(75, 0.90);
        transfer.add(83, 0.89);
        transfer.add(91, 0.89);
        transfer.add(99, 0.89);
        transfer.add(107, 0.89);
        transfer.add(118, 0.88);
        transfer.add(131, 0.8);
        transfer.add(147, 0.45);
        transfer.add(3000, 0.45);
        transfer.createLUT();



        shottime.add(0, 0.7);
        shottime.add(59, 0.7);
        shottime.add(67, 0.76);
        shottime.add(75, 0.64);
        shottime.add(83, 0.7);
        shottime.add(91, 0.72);
        shottime.add(99, 0.83);
        shottime.add(107, 0.92);
        shottime.add(118, 0.9);
        shottime.add(131, 0.92);
        shottime.add(147, 1.02);
        shottime.add(3000, 0.6);
        shottime.createLUT();

        /*

        distance, rad, hood, transferspeed
        29, 275, 1, 0.55
        35.5, 275, 0.7, 0.7
        43, 275, 0.7, 1
        49.5, 285, 0.65, 1
        56.5, 290. 0.58, 1
        68, 310, 0.5, 1
        76, 315, 0.45, 1
        83, 335, 0.40, 1
        91, 360, 0.30, 1
         */
    }

    // --- Public commands ---

    public Command turretOff (boolean off) {
        return new InstantCommand(() -> turretOff = off ? 0 : 1);
    }

    public void startTurret() {
        turretOff = 0;
        this.periodic();

    }

    public void flywheelOff() {
        flywheelOn = false;

    }

    public Command flywheel(boolean on) {
        return new InstantCommand(() -> flywheelOn = on);
    }
    public Command aimAt(Pose shootPose, double heading) {
        double finalDeg = computeTurretAngle(shootPose, heading);
        return new InstantCommand(() -> fixedAngleDeg = finalDeg);
    }
    /** Direct setter for use in initialize() (not as a scheduled command). */
    public void aimAtNow(Pose shootPose, double heading) {
        fixedAngleDeg = computeTurretAngle(shootPose, heading);
    }
    public Command clearFixedAngle() {
        return new InstantCommand(() -> fixedAngleDeg = null);
    }
    private double computeTurretAngle(Pose shootPose, double heading) {
        double dx = shooterX - shootPose.getX();
        double dy = shooterY - shootPose.getY();
        double fieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretDeg = fieldDeg - Math.toDegrees(heading);
        while (turretDeg > 180) turretDeg -= 360;
        while (turretDeg < -180) turretDeg += 360;
        return turretDeg;
    }
    public Command flywheelToggle() {
        return new InstantCommand(() -> flywheelOn = !flywheelOn);
    }

    public Command SWMToggle() {
        return new InstantCommand(() -> SWM = !SWM);
    }
    public Command SWMon() {
        return new InstantCommand(() -> SWM = true);
    }
    public Command SWMoff() {
        return new InstantCommand(() -> SWM = false);
    }

    /** Returns current flywheel RPM (bottom motor velocity converted from rad/s). */
    /** Returns flywheel velocity in rad/s (same units as PID target). */
    public double getFlywheelRpm() {
        return shooterB.getVelocity() * (2 * Math.PI / 28);
    }

    public Command increaseTurretOffset() {
        return new InstantCommand(() -> turretOffset += 5);
    }

    public Command decreaseTurretOffset() {
        return new InstantCommand(() -> turretOffset -= 5);
    }

    public Command increaseHoodOffset() {
        return new InstantCommand(() -> hoodOffset += 0.05);
    }

    public Command decreaseHoodOffset() {
        return new InstantCommand(() -> hoodOffset -= 0.05);
    }

    public Command offsetZero() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> hoodOffset = 0),
                new InstantCommand(() -> turretOffset = 0)
        );
    }

    // --- Absolute encoder ---

    public double getAbsAngle() {
        if (!ABS_ENABLED || abs == null) return Double.NaN;
        return abs.getVoltage() * m + b;
    }

    // --- Coordinate conversion ---

    /** Turret degrees → servo position [SERVO_MIN, SERVO_MAX] */
    private double turretDegToServoPos(double turretDeg) {
        double pos = SERVO_CENTER - turretDeg / (SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO);
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, pos));
    }

    /** Servo position [0, 1] → turret degrees */
    private double servoPosToTurretDeg(double servoPos) {
        return -(servoPos - SERVO_CENTER) * SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO;
    }

    /** Full speed in the middle, linearly reduces near either mechanical limit. */
    private double computeMaxStep() {
        double currentDeg = servoPosToTurretDeg(currentServoPos);
        double distToEdge = Math.min(currentDeg - TURRET_MIN, TURRET_MAX - currentDeg);

        if (distToEdge >= EDGE_ZONE_DEG) {
            return MAX_SERVO_STEP;
        }
        double t = 1.0 - Math.max(0.0, distToEdge / EDGE_ZONE_DEG);
        return MAX_SERVO_STEP + t * (MIN_SERVO_STEP - MAX_SERVO_STEP);
    }

    // --- Fused position update (abs encoder only — no motor encoder in V2) ---

    private void updateFusedPosition() {
        double dt = timer.seconds();
        timer.reset();

        if (!ABS_ENABLED || abs == null) {
            // No abs encoder — track commanded servo position only
            fusedTurretPos = servoPosToTurretDeg(currentServoPos);
            turretPosDeg = fusedTurretPos;
            return;
        }

        double realPos = getAbsAngle();

        // Complementary filter correction toward abs encoder
        double k;
        if (TURRET_TAU <= 1e-6 || dt <= 0.0) {
            k = dt <= 0.0 ? 0.0 : 1.0;
        } else {
            k = 1.0 - Math.exp(-dt / TURRET_TAU);
        }
        k = Math.max(0.0, Math.min(1.0, k));

        double absError = angleDiffDeg(realPos, fusedTurretPos);
        if (Math.abs(absError) > CORRECTION_DEADBAND) {
            fusedTurretPos += k * absError;
        }

        fusedTurretPos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, fusedTurretPos));
        turretPosDeg = fusedTurretPos;

        Log.d("TurretPosAbs", String.valueOf(realPos));
        Log.d("TurretPosFused", String.valueOf(fusedTurretPos));
    }

    private double angleDiffDeg(double a, double b) {
        double diff = a - b;
        return ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    public double getServoPos(){
        return currentServoPos;
    }

    private void setTurretDeg(double targetTurretDeg) {
        targetTurretDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTurretDeg));
        double targetServoPos = turretDegToServoPos(targetTurretDeg);

        if (DIRECT_WRITE) {
            // Write target directly — servo manages its own movement.
            // Only slew-limit near mechanical stops if EDGE_ZONE_ENABLED.
            double currentDeg = servoPosToTurretDeg(currentServoPos);
            double distToEdge = Math.min(currentDeg - TURRET_MIN, TURRET_MAX - currentDeg);

            if (EDGE_ZONE_ENABLED && distToEdge < EDGE_ZONE_DEG) {
                double t = 1.0 - Math.max(0.0, distToEdge / EDGE_ZONE_DEG);
                double maxStep = MAX_SERVO_STEP + t * (MIN_SERVO_STEP - MAX_SERVO_STEP);
                double error = targetServoPos - currentServoPos;
                if (Math.abs(error) > maxStep) {
                    currentServoPos += Math.signum(error) * maxStep;
                } else {
                    currentServoPos = targetServoPos;
                }
            } else {
                currentServoPos = targetServoPos;
            }
        } else {
            // Full slew limiting every step
            double maxStep = computeMaxStep();
            double error = targetServoPos - currentServoPos;
            if (Math.abs(error) > maxStep) {
                currentServoPos += Math.signum(error) * maxStep;
            } else {
                currentServoPos = targetServoPos;
            }
        }

        // Skip write if position unchanged
        if (hasAppliedServoPos && Double.compare(lastAppliedServoPos, currentServoPos) == 0) {
            return;
        }
        lastAppliedServoPos = currentServoPos;
        hasAppliedServoPos = true;

        turretL1.set(currentServoPos + SERVO_OFFSET_TL1);
        turretL2.set(currentServoPos + SERVO_OFFSET_TL2);
        turretR1.set(currentServoPos + SERVO_OFFSET_TR1);
    }

    // --- Periodic ---

    @Override
    public void periodic() {
        // updateFusedPosition();

        if (Memory.debugMode) {
            loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
        }

        Pose robot = followerSupplier.get().getPose();
        double presentVoltage = volt.getVoltage();

        batteryVoltage = presentVoltage;

        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();
        robotHeadingDeg = Math.toDegrees(robotHeading);
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretX = TURRET_FWD_OFFSET * cosH;
        double turretY = TURRET_FWD_OFFSET * sinH;

        // Shoot-while-moving: 10-iteration solver (carry over from V1)
        // TODO: Uncomment after basic turret tracking + ENABLE_OMEGA_COMP are confirmed working.
        //       When re-enabling, also add heading prediction:
        //         double headingForAngle = robotHeading;
        //         headingForAngle = robotHeading + rawOmegaRad * shotTime;
        //         turretX = TURRET_FWD_OFFSET * Math.cos(headingForAngle);
        //         turretY = TURRET_FWD_OFFSET * Math.sin(headingForAngle);
        //       And use headingForAngle instead of robotHeading in the atan2 subtraction below.
        double dx = shooterX - robotX - turretX;
        double dy = shooterY - robotY - turretY;
        double distance = Math.sqrt(dx * dx + dy * dy);
//        Log.d("Distance", String.valueOf(distance));
        if (SWM == true) {
            for (int i = 0; i < 3; i++) {
                double shotTime = shottime.get(distance);
                double vX = followerSupplier.get().getVelocity().getXComponent();
                double vY = followerSupplier.get().getVelocity().getYComponent();
                dx = shooterX - robotX - vX * shotTime - turretX;
                dy = shooterY - robotY - vY * shotTime - turretY;
                Log.d("Velocity " + String.valueOf(i), String.valueOf(Math.sqrt(vX * vX + vY * vY)));
                if (Math.sqrt(vX * vX + vY * vY) > 7)
                    distance = Math.sqrt(dx * dx + dy * dy);
                if (Memory.debugMode) {
                    Log.d("Distance " + String.valueOf(i), String.valueOf(distance));
                }
            }
        }

        // Turret angle target
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(robotHeading);

        if (fixedAngleDeg != null) {
            targetAngleDeg = fixedAngleDeg;
        }

        targetAngleDeg *= turretOff;
        targetAngleDeg += turretOffset;

        // Angular velocity compensation — leads the turret command to compensate for servo lag
        if (ENABLE_OMEGA_COMP) {
            servoLagCompDeg = -filteredOmegaDeg * SERVO_T_LAG;
            servoLagCompDeg = Math.max(-OMEGA_COMP_CLAMP, Math.min(OMEGA_COMP_CLAMP, servoLagCompDeg));
            targetAngleDeg += servoLagCompDeg;
        } else {
            servoLagCompDeg = 0;
        }

        turretTargetDeg = targetAngleDeg;

        // Pick the ±360 candidate closest to lastChosenDeg that's within turret range
        double[] cands = { targetAngleDeg, targetAngleDeg + 360.0, targetAngleDeg - 360.0 };
        double chosen = lastChosenDeg; // fallback: hold position if nothing is in range
        double bestDist = Double.MAX_VALUE;
        for (double cand : cands) {
            if (cand >= TURRET_MIN && cand <= TURRET_MAX) {
                double dist = Math.abs(cand - lastChosenDeg);
                if (dist < bestDist) {
                    bestDist = dist;
                    chosen = cand;
                }
            }
        }

        lastChosenDeg = chosen;

        setTurretDeg(chosen);
        turretPosDeg = servoPosToTurretDeg(currentServoPos);

        if (currentServoPos <= SERVO_MIN + 0.01 || currentServoPos >= SERVO_MAX - 0.01) {
            if (Memory.debugMode) {
                Log.w("Turret", String.format(
                        "SERVO NEAR LIMIT: servoPos=%.4f (min=%.2f max=%.2f)",
                        currentServoPos, SERVO_MIN, SERVO_MAX));
            }
        }

        // Hood
        if (HOOD_ENABLED && hood != null) {
            double theta = Math.max(0, Math.min(1, angle.get(distance) + hoodOffset));
            hood.setPosition(theta);
        }

        // Update transfer speed target for Intake to read
        transferSpeedTarget = transfer.get(distance);

        // Flywheel
        if (flywheelOn) {
            if (MANUAL_POWER > 0) {
                // Raw power mode — use during initial testing before PID is tuned
                shooterB.set(MANUAL_POWER);
                shooterT.set(MANUAL_POWER);
            } else {
                // PID + FF mode — target from MANUAL_RPM override or distance LUT
                double target = MANUAL_RPM > 0 ? MANUAL_RPM : RPM.get(distance);
                double vel = shooterB.getVelocity() * (2 * Math.PI / 28);  // rad/s — matches V1
                controllerShooter.setPID(p, i, d);
                double pidVolts = controllerShooter.calculate(vel, target);
                pidVolts = Math.max(-presentVoltage, Math.min(pidVolts, presentVoltage));

                double ffVolts = ENABLE_FF ? (kV * target + kS * Math.signum(target)) : f * target;
                double flywheelVolts = Math.max(-presentVoltage, Math.min(pidVolts + ffVolts, presentVoltage));

                shooterB.set(flywheelVolts / presentVoltage);
                shooterT.set(flywheelVolts / presentVoltage);
            }
        } else {
            shooterB.set(0);
            shooterT.set(0);
        }
        Log.d("Distance", String.valueOf(distance));
        /*
        Log.d("Distance", String.valueOf(distance));
        Log.d("FlywheelRPM", String.valueOf(shooterB.getVelocity() / 28.0 * 60.0));
        Log.d("TargetRPM", String.valueOf(RPM.get(distance)));
        */
    }
}
