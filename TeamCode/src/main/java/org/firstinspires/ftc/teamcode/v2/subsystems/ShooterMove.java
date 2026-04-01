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
    public static double TURRET_MIN = -130.0;
    public static double TURRET_MAX = 255.0;

    // --- Turret servo geometry ---
    // Axon MAX: 355° range. Gear ratio: 48t:15t × 47t:107t = 1.408x
    public static double SERVO_RANGE_DEG = 355.0;
    public static double SERVO_TO_TURRET_RATIO = (48.0 / 15.0) * (47.0 / 107.0); // ~1.408
    public static double SERVO_CENTER = 0.5; // tune on real robot — center of turret travel
    // Safe servo limits — 0.03/0.97 avoids physical endpoints (~30° margin each side)
    // Usable servo range: 333.7° → turret range: ~470° through 1.408x gear ratio
    public static double SERVO_MIN = 0.03;
    public static double SERVO_MAX = 0.97;

    // --- Abs encoder ---
    public static boolean ABS_ENABLED = false;

    // --- Hood ---
    public static boolean HOOD_ENABLED = false;

    // --- r1 direction: all three servos confirmed same direction ---

    // --- Slew rate limiting — prevents slamming mechanical stops ---
    public static double MAX_SERVO_STEP = 0.05;  // max position change per loop at full speed
    public static double MIN_SERVO_STEP = 0.01;  // min step near limits
    public static double EDGE_ZONE_DEG  = 20.0;  // degrees from limit where slew starts reducing

    // --- Flywheel PID + FF (carry over from V1 — retune on V2) ---
    public static double p = 0.8, i = 0.05, d = 0;
    public static boolean ENABLE_FF = false;
    public static double kV = 0.0211771178235103;
    public static double kS = 0.461428918657443;
    public static double f = 0.0265;
    // Set > 0 to override LUT and run at a fixed RPM target (for manual testing)
    public static double MANUAL_RPM = 0;
    // Set 0.0–1.0 to drive flywheel at raw power (bypasses PID — use for initial testing)
    public static double MANUAL_POWER = 0.5;

    // --- Turret offset adjustment ---
    public static double turretOffset = 0;

    // --- Turret position (published for telemetry / other subsystems) ---
    public static double turretPosDeg = 0;

    // --- Shooter position on field (set by teleop, same as V1) ---
    public static double TURRET_FWD_OFFSET = -1.63;
    public static double TURRET_LEFT_OFFSET = 0.0;

    // Hardware
    private final ServoEx turretL1, turretL2, turretR1;
    private final ServoEx hood;  // null when HOOD_ENABLED = false
    private final MotorEx shooterB, shooterT;
    private final AnalogInput abs;  // null when ABS_ENABLED = false
    private final VoltageSensor volt;
    private final Supplier<Follower> followerSupplier;

    // State
    private boolean flywheelOn = true;
    private double hoodOffset = 0;
    private final PIDController controllerShooter;
    private double shooterX, shooterY;
    private double currentServoPos;
    private boolean hasAppliedServoPos = false;
    private double lastAppliedServoPos = SERVO_CENTER;

    // Fused turret position (abs encoder only — no motor encoder in V2)
    private double fusedTurretPos = 0.0;
    private ElapsedTime timer;
    private static final double TURRET_TAU = 0.1;
    private static final double CORRECTION_DEADBAND = 0.05;

    // LUTs (carry over from V1 — retune on V2)
    InterpLUT RPM      = new InterpLUT();
    InterpLUT angle    = new InterpLUT();
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
        hood     = HOOD_ENABLED ? new ServoEx(hMap, "hood") : null;
        shooterB = new MotorEx(hMap, "sb");
        shooterT = new MotorEx(hMap, "st");
        abs      = ABS_ENABLED ? hMap.get(AnalogInput.class, "abs") : null;
        volt     = hMap.get(VoltageSensor.class, "Control Hub");

        shooterB.setRunMode(MotorEx.RunMode.RawPower);
        shooterT.setRunMode(MotorEx.RunMode.RawPower);

        // Initialize servo position — use abs encoder if available, else start at 0°
        fusedTurretPos = ABS_ENABLED ? getAbsAngle() : 0.0;
        currentServoPos = turretDegToServoPos(fusedTurretPos);

        timer = new ElapsedTime();
        timer.reset();

        controllerShooter = new PIDController(p, i, d);

        // LUTs — V1 values, retune on V2
        RPM.add(0, 310);
        RPM.add(42.5, 280);
        RPM.add(49.5, 300);
        RPM.add(56.5, 320);
        RPM.add(70, 340);
        RPM.add(77.25, 350);
        RPM.add(91.75, 370);
        RPM.add(102.75, 390);
        RPM.add(114, 415);
        RPM.add(130.75, 445);
        RPM.add(142, 470);
        RPM.add(3000, 485);
        RPM.createLUT();

        angle.add(0, 0.6);
        angle.add(42.5, 0.7);
        angle.add(49.5, 0.7);
        angle.add(56.5, 0.45);
        angle.add(70, 0.21);
        angle.add(77.25, 0.22);
        angle.add(91.75, 0.15);
        angle.add(102.75, 0.12);
        angle.add(114, 0.10);
        angle.add(130.75, 0.03);
        angle.add(142, 0.1);
        angle.add(3000, 0.01);
        angle.createLUT();

        shottime.add(0, 0.63);
        shottime.add(42.5, 0.53);
        shottime.add(55, 0.41);
        shottime.add(66.7, 0.45);
        shottime.add(81.9, 0.55);
        shottime.add(95.7, 0.67);
        shottime.add(101.9, 0.7);
        shottime.add(116.6, 0.72);
        shottime.add(136.6, 0.95);
        shottime.add(3000, 1);
        shottime.createLUT();
    }

    // --- Public commands ---

    public Command flywheel(boolean on) {
        return new InstantCommand(() -> flywheelOn = on);
    }

    public Command flywheelToggle() {
        return new InstantCommand(() -> flywheelOn = !flywheelOn);
    }

    /** Returns current flywheel RPM (bottom motor velocity converted from rad/s). */
    public double getFlywheelRpm() {
        return shooterB.getVelocity() * (60.0 / 28.0);
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
        double pos = SERVO_CENTER + turretDeg / (SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO);
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, pos));
    }

    /** Servo position [0, 1] → turret degrees */
    private double servoPosToTurretDeg(double servoPos) {
        return (servoPos - SERVO_CENTER) * SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO;
    }

    // --- Slew-rate-limited servo write ---

    private void setTurretDeg(double targetTurretDeg) {
        targetTurretDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTurretDeg));
        double targetServoPos = turretDegToServoPos(targetTurretDeg);

        double maxStep = computeMaxStep(targetTurretDeg);
        double error = targetServoPos - currentServoPos;
        if (Math.abs(error) > maxStep) {
            currentServoPos += Math.signum(error) * maxStep;
        } else {
            currentServoPos = targetServoPos;
        }

        // Skip write if position unchanged
        if (hasAppliedServoPos && Double.compare(lastAppliedServoPos, currentServoPos) == 0) {
            return;
        }
        lastAppliedServoPos = currentServoPos;
        hasAppliedServoPos = true;

        turretL1.set(currentServoPos);
        turretL2.set(currentServoPos);
        turretR1.set(currentServoPos);
    }

    /** Full speed in the middle, linearly reduces near mechanical limits. */
    private double computeMaxStep(double targetTurretDeg) {
        double absAngle = Math.abs(targetTurretDeg);
        double edgeZoneStart = Math.max(Math.abs(TURRET_MIN), Math.abs(TURRET_MAX)) - EDGE_ZONE_DEG;

        if (absAngle <= edgeZoneStart) {
            return MAX_SERVO_STEP;
        }
        double edgeZoneEnd = Math.max(Math.abs(TURRET_MIN), Math.abs(TURRET_MAX));
        double t = (absAngle - edgeZoneStart) / (edgeZoneEnd - edgeZoneStart);
        t = Math.max(0.0, Math.min(1.0, t));
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

    // --- Periodic ---

    @Override
    public void periodic() {
        updateFusedPosition();

        Pose robot = followerSupplier.get().getPose();
        double presentVoltage = volt.getVoltage();

        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretX = TURRET_FWD_OFFSET * cosH;
        double turretY = TURRET_FWD_OFFSET * sinH;

        // Shoot-while-moving: 10-iteration solver (carry over from V1)
        double dx = shooterX - robotX - turretX;
        double dy = shooterY - robotY - turretY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        for (int i = 0; i < 10; i++) {
            double shotTime = shottime.get(distance);
            double vX = followerSupplier.get().getVelocity().getXComponent();
            double vY = followerSupplier.get().getVelocity().getYComponent();
            dx = shooterX - robotX - vX * shotTime - turretX;
            dy = shooterY - robotY - vY * shotTime - turretY;
            distance = Math.sqrt(dx * dx + dy * dy);
        }

        // Turret angle target
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx)) - Math.toDegrees(robotHeading);
        targetAngleDeg += turretOffset;

        // Pick best candidate within limits (handles wrap-around)
        double[] cands = { targetAngleDeg, targetAngleDeg + 360.0, targetAngleDeg - 360.0 };
        double chosen = targetAngleDeg;
        double bestDist = Double.MAX_VALUE;
        for (double cand : cands) {
            if (cand >= TURRET_MIN && cand <= TURRET_MAX) {
                double dist = Math.abs(cand - fusedTurretPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    chosen = cand;
                }
            }
        }
        chosen = Math.max(TURRET_MIN, Math.min(TURRET_MAX, chosen));

        setTurretDeg(chosen);
        Log.d("TurretTarget", String.valueOf(chosen));

        // Hood
        if (HOOD_ENABLED && hood != null) {
            double theta = Math.max(0, Math.min(1, angle.get(distance) + hoodOffset));
            hood.set(theta);
        }

        // Flywheel
        if (flywheelOn) {
            if (MANUAL_POWER > 0) {
                // Raw power mode — use during initial testing before PID is tuned
                shooterB.set(-MANUAL_POWER);
                shooterT.set(MANUAL_POWER);
            } else {
                // PID + FF mode — target from MANUAL_RPM override or distance LUT
                double target = MANUAL_RPM > 0 ? MANUAL_RPM : RPM.get(distance);
                double vel = shooterB.getVelocity() / 28.0 * 60.0;  // ticks/sec → RPM
                controllerShooter.setPID(p, i, d);
                double pidVolts = controllerShooter.calculate(vel, target);
                pidVolts = Math.max(-presentVoltage, Math.min(pidVolts, presentVoltage));

                double ffVolts = ENABLE_FF ? (kV * target + kS * Math.signum(target)) : f * target;
                double flywheelVolts = Math.max(-presentVoltage, Math.min(pidVolts + ffVolts, presentVoltage));

                shooterB.set((-1) * flywheelVolts / presentVoltage);
                shooterT.set(flywheelVolts / presentVoltage);
            }
        } else {
            shooterB.set(0);
            shooterT.set(0);
        }

        Log.d("Distance", String.valueOf(distance));
        Log.d("FlywheelRPM", String.valueOf(shooterB.getVelocity() / 28.0 * 60.0));
    }
}
