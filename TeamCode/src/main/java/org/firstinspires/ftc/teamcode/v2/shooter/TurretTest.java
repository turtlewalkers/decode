package org.firstinspires.ftc.teamcode.v2.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

/**
 * Turret servo + absolute encoder verification opmode.
 * Two modes selectable via START button:
 *
 * === MODE 1: MANUAL ===
 * Direct servo position control. Use to:
 *   - Verify abs encoder calibration (m, b) — should read ~0° at mechanical center
 *   - Find SERVO_CENTER — servoPos where turret faces forward
 *   - Find TURRET_MIN / TURRET_MAX — abs encoder reading at each mechanical stop
 *
 * Controls:
 *   DPAD_RIGHT/LEFT: +/- STEP_SIZE
 *   A:  go to SERVO_CENTER
 *   B:  go to 0.03 (servo min)
 *   Y:  go to 0.97 (servo max)
 *   LB: decrease STEP_SIZE (finer)
 *   RB: increase STEP_SIZE (coarser)
 *   START: switch to SLEW mode
 *
 * === MODE 2: SLEW TEST ===
 * Feeds TARGET_DEG through the same slew-rate + soft-limit logic as ShooterMove.
 * Use to tune MAX_SERVO_STEP, MIN_SERVO_STEP, EDGE_ZONE_DEG, TURRET_MIN, TURRET_MAX
 * before wiring up ShooterMove.
 *
 * Procedure:
 *   1. Set TARGET_DEG on Dashboard to a large jump (e.g. TURRET_MIN to TURRET_MAX).
 *      Watch turret slow down near limits — tune EDGE_ZONE_DEG.
 *   2. Tune MAX_SERVO_STEP for overall speed, MIN_SERVO_STEP for limit approach speed.
 *   3. Set TARGET_DEG past TURRET_MIN/MAX — confirm soft limits hold.
 *   4. Copy confirmed values to ShooterMove.java.
 *
 * Controls:
 *   A:  set TARGET_DEG = 0 (center)
 *   B:  set TARGET_DEG = TURRET_MIN
 *   Y:  set TARGET_DEG = TURRET_MAX
 *   START: switch back to MANUAL mode
 */
@Config
@TeleOp(name = "TurretTest", group = "V2")
public class TurretTest extends LinearOpMode {

    // --- Abs encoder calibration ---
    public static boolean ABS_ENABLED = false;
    public static double m = -123.71;
    public static double b = 256.37;

    // --- Servo geometry (copy confirmed values to ShooterMove) ---
    public static double SERVO_CENTER = 0.525;
    public static double SERVO_MIN    = 0.03;
    public static double SERVO_MAX    = 0.97;
    public static double SERVO_RANGE_DEG        = 355.0;
    public static double SERVO_TO_TURRET_RATIO  = (48.0 / 15.0) * (47.0 / 107.0); // ~1.408

    // --- Turret soft limits in turret degrees (tune here, copy to ShooterMove) ---
    public static double TURRET_MIN = -140.0; //-130.0
    public static double TURRET_MAX =  245.0; //-255.0

    // --- Slew rate limiting (tune here, copy to ShooterMove) ---
    public static double MAX_SERVO_STEP = 0.05;  // max position change per loop (~20ms)
    public static double MIN_SERVO_STEP = 0.01;  // step near mechanical limits
    public static double EDGE_ZONE_DEG  = 20.0;  // degrees from limit where slew starts reducing

    // --- Slew test target (set from Dashboard in SLEW mode) ---
    public static double TARGET_DEG = 0.0;

    // --- Manual mode step size ---
    public static double STEP_SIZE = 0.01;

    private static final String TAG = "TurretTest";

    // Internal slew state
    private double currentServoPos = 0.5;
    private boolean lastApplied = false;
    private double lastServoPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoEx l1 = new ServoEx(hardwareMap, "tl1");
        ServoEx l2 = new ServoEx(hardwareMap, "tl2");
        ServoEx r1 = new ServoEx(hardwareMap, "tr1");
        AnalogInput abs = ABS_ENABLED
                ? hardwareMap.get(AnalogInput.class, "abs")
                : null;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Axon MAX requires 500–2500µs PWM range (FTC default is 750–2250µs)
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);
        ((PwmControl) hardwareMap.get(Servo.class, "tl1")).setPwmRange(axonRange);
        ((PwmControl) hardwareMap.get(Servo.class, "tl2")).setPwmRange(axonRange);
        ((PwmControl) hardwareMap.get(Servo.class, "tr1")).setPwmRange(axonRange);

       // currentServoPos = SERVO_CENTER;
        // Read actual servo position - Axon max is absolute, always knows where it is
        currentServoPos = l1.getRawPosition();
        applyServoPos(l1, l2, r1, currentServoPos);

        telemetry.addLine("TurretTest ready.  START = toggle MANUAL / SLEW mode");
        telemetry.update();

        boolean slewMode = false;
        boolean lastRight = false, lastLeft = false, lastA = false;
        boolean lastB = false, lastY = false, lastLB = false, lastRB = false;
        boolean lastStart = false;

        double currentServoPosl1 = l1.getRawPosition();//SERVO_CENTER;
        double currentServoPosl2 = l1.getRawPosition(); //SERVO_CENTER;
        double currentServoPosr1 = l1.getRawPosition(); //SERVO_CENTER;

        waitForStart();

        while (opModeIsActive()) {
            boolean right = gamepad1.dpad_right;
            boolean left  = gamepad1.dpad_left;
            boolean a     = gamepad1.a;
            boolean bBtn  = gamepad1.b;
            boolean y     = gamepad1.y;
            boolean lb    = gamepad1.left_bumper;
            boolean rb    = gamepad1.right_bumper;
            boolean start = gamepad1.start;

            // Toggle mode
            if (start && !lastStart) {
                slewMode = !slewMode;
                // Sync slew state to current position on mode switch
                currentServoPos = servoPosFromTurretDeg(
                        Math.max(TURRET_MIN, Math.min(TURRET_MAX, TARGET_DEG)));
                lastApplied = false;
            }
            lastStart = start;

            double absVolts = (ABS_ENABLED && abs != null) ? abs.getVoltage() : Double.NaN;
            double absDeg   = (ABS_ENABLED && abs != null) ? absVolts * m + b : Double.NaN;

            if (!slewMode) {
                // --- MANUAL MODE ---
                if (right && !lastRight) {

                    currentServoPosl1 = Math.min(SERVO_MAX, currentServoPos + STEP_SIZE);
                    currentServoPosl2 = Math.min(SERVO_MAX, currentServoPos + STEP_SIZE);
                    currentServoPosr1 = Math.min(SERVO_MAX, currentServoPos + STEP_SIZE);
                }
                if (left  && !lastLeft)  {
                    currentServoPosl1 = Math.max(SERVO_MIN, currentServoPos - STEP_SIZE);
                    currentServoPosl2 = Math.max(SERVO_MIN, currentServoPos - STEP_SIZE);
                    currentServoPosr1 = Math.max(SERVO_MIN, currentServoPos - STEP_SIZE);
                }
                if (a     && !lastA) {
                    currentServoPosl1 = SERVO_CENTER;
                    currentServoPosl2 = SERVO_CENTER;
                    currentServoPosr1 = SERVO_CENTER;
                }
                if (bBtn  && !lastB)    {
                    //Keep this for later for zeroing the servos
                   // currentServoPosl1 = SERVO_MIN + 0.002;
                   // currentServoPosl2 = SERVO_MIN;
                   // currentServoPosr1 = SERVO_MIN - 0.002;

                    currentServoPosl1 = SERVO_MIN;
                    currentServoPosl2 = SERVO_MIN;
                    currentServoPosr1 = SERVO_MIN;

                }
                if (y     && !lastY)  {
                    currentServoPosl1 = SERVO_MAX;
                    currentServoPosl2 = SERVO_MAX;
                    currentServoPosr1 = SERVO_MAX;
                }
                if (lb    && !lastLB)    STEP_SIZE = Math.max(0.005, STEP_SIZE - 0.005);
                if (rb    && !lastRB)    STEP_SIZE = Math.min(0.05,  STEP_SIZE + 0.005);

                applyServoPosl1(l1, currentServoPosl1);
                applyServoPosl2(l2, currentServoPosl2);
                applyServoPosr1(r1, currentServoPosr1);

                double turretDeg = turretDegFromServoPos(currentServoPos);
                telemetry.addLine("=== MANUAL MODE === (START to switch)");
                telemetry.addData("servoPosl1",          "%.4f", currentServoPosl1);
                telemetry.addData("servoPosl2",          "%.4f", currentServoPosl2);
                telemetry.addData("servoPosr1",          "%.4f", currentServoPosr1);
                telemetry.addData("Turret deg (calc)",  "%.1f", turretDeg);
                if (ABS_ENABLED) {
                    telemetry.addData("Abs Encoder (V)",   "%.4f", absVolts);
                    telemetry.addData("Abs Encoder (deg)", "%.1f", absDeg);
                } else {
                    telemetry.addData("Abs Encoder", "DISABLED");
                }
                telemetry.addData("SERVO_CENTER",       "%.4f", SERVO_CENTER);
                telemetry.addData("STEP_SIZE",          "%.3f", STEP_SIZE);
                telemetry.addLine("B=min(0.03)  Y=max(0.97)  A=center");
                telemetry.addLine("At limits: note Abs deg → TURRET_MIN/MAX");

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("mode", 0);
                packet.put("servoPosl1",       currentServoPosl1);
                packet.put("servoPosl2",       currentServoPosl2);
                packet.put("servoPosr1",       currentServoPosr1);
                packet.put("turretDeg_calc", turretDeg);
                packet.put("absEncoder_deg", absDeg);
                packet.put("SERVO_CENTER",   SERVO_CENTER);
                dashboard.sendTelemetryPacket(packet);

            } else {
                // --- SLEW TEST MODE ---
                if (a    && !lastA)  TARGET_DEG = 0;
                if (bBtn && !lastB)  TARGET_DEG = TURRET_MIN;
                if (y    && !lastY)  TARGET_DEG = TURRET_MAX;

               // setTurretDeg(TARGET_DEG);
                double targetServoPos = servoPosFromTurretDeg(TARGET_DEG);
                applyServoPos(l1, l2, r1, targetServoPos);
                currentServoPos = targetServoPos;
                double turretDeg = turretDegFromServoPos(currentServoPos);
                double error = TARGET_DEG - turretDeg;
                telemetry.addLine("=== SLEW MODE === (START to switch)");
                telemetry.addData("TARGET_DEG",         "%.1f", TARGET_DEG);
                telemetry.addData("servoPos",           "%.4f", currentServoPos);
                telemetry.addData("Turret deg (calc)",   "%.1f", turretDeg);
                telemetry.addData("Error (deg)",         "%.1f", error);
                if (ABS_ENABLED) {
                    telemetry.addData("Abs Encoder (deg)", "%.1f", absDeg);
                } else {
                    telemetry.addData("Abs Encoder", "DISABLED");
                }
                telemetry.addData("MAX_SERVO_STEP",      "%.4f", MAX_SERVO_STEP);
                telemetry.addData("MIN_SERVO_STEP",      "%.4f", MIN_SERVO_STEP);
                telemetry.addData("EDGE_ZONE_DEG",       "%.1f", EDGE_ZONE_DEG);
                telemetry.addData("TURRET_MIN",          "%.1f", TURRET_MIN);
                telemetry.addData("TURRET_MAX",          "%.1f", TURRET_MAX);
                telemetry.addLine("B=go to MIN  Y=go to MAX  A=go to 0");

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("mode", 1);
                packet.put("TARGET_DEG",     TARGET_DEG);
                packet.put("servoPos",       currentServoPos);
                packet.put("turretDeg_calc", turretDeg);
                packet.put("error_deg",      error);
                packet.put("absEncoder_deg", absDeg);
                packet.put("maxStep",        computeMaxStep(turretDeg));
                dashboard.sendTelemetryPacket(packet);
            }

            lastRight = right; lastLeft = left; lastA = a;
            lastB = bBtn; lastY = y; lastLB = lb; lastRB = rb;

            RobotLog.dd(TAG, "mode=%s servoPos=%.4f absV=%.4f absDeg=%.1f",
                    slewMode ? "SLEW" : "MANUAL", currentServoPos, absVolts, absDeg);

            telemetry.update();
            sleep(20);
        }
    }

    // --- Slew logic (mirrors ShooterMove.setTurretDeg) ---

    private void setTurretDeg(double targetTurretDeg) {
        targetTurretDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTurretDeg));
        double targetServoPos = servoPosFromTurretDeg(targetTurretDeg);

        double maxStep = computeMaxStep(turretDegFromServoPos(currentServoPos));
        double error = targetServoPos - currentServoPos;
        if (Math.abs(error) > maxStep) {
            currentServoPos += Math.signum(error) * maxStep;
        } else {
            currentServoPos = targetServoPos;
        }
        currentServoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, currentServoPos));
    }

    private double computeMaxStep(double currentTurretDeg) {
        double absAngle = Math.abs(currentTurretDeg);
        double edgeZoneStart = Math.max(Math.abs(TURRET_MIN), Math.abs(TURRET_MAX)) - EDGE_ZONE_DEG;

        if (absAngle <= edgeZoneStart) {
            return MAX_SERVO_STEP;
        }
        double edgeZoneEnd = Math.max(Math.abs(TURRET_MIN), Math.abs(TURRET_MAX));
        double t = (absAngle - edgeZoneStart) / (edgeZoneEnd - edgeZoneStart);
        t = Math.max(0.0, Math.min(1.0, t));
        return MAX_SERVO_STEP + t * (MIN_SERVO_STEP - MAX_SERVO_STEP);
    }

    // --- Coordinate helpers ---

    private double servoPosFromTurretDeg(double turretDeg) {
        double pos = SERVO_CENTER - turretDeg / (SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO);
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, pos));
    }

    private double turretDegFromServoPos(double pos) {
        return -(pos - SERVO_CENTER) * SERVO_RANGE_DEG * SERVO_TO_TURRET_RATIO;
    }

    // --- Servo write ---

    private void applyServoPos(ServoEx l1, ServoEx l2, ServoEx r1, double pos) {
        l1.set(pos);
        l2.set(pos);
        r1.set(pos);
    }
    private void applyServoPosl1(ServoEx l1, double pos) {
        l1.set(pos);
    }
    private void applyServoPosl2(ServoEx l2, double pos) {
        l2.set(pos);
    }
    private void applyServoPosr1(ServoEx r1, double pos) {
        r1.set(pos);
    }
}
