package org.firstinspires.ftc.teamcode.v2.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Drive motor diagnostic opmode.
 *
 * Same stick controls as TeleopMoving (left stick Y = forward, left stick X = strafe,
 * right stick X = turn) but drives motors directly — no Pedro, no follower, no odometry.
 *
 * Use this to isolate mecanum drift. Watch Dashboard for per-wheel current and velocity.
 * A weak or slipping wheel shows lower current than its diagonal partner at the same command.
 *
 * === DRIFT DIAGNOSIS ===
 * Drifts RIGHT when going forward → suspect LB or RF (same roller diagonal)
 * Drifts LEFT  when going forward → suspect LF or RB (same roller diagonal)
 * Watch current: low current on a wheel = slipping / loose hub / bad connection
 * Watch velocity: low velocity on a wheel = mechanical drag or encoder issue
 *
 * === INDIVIDUAL WHEEL TEST ===
 * Set SINGLE_WHEEL_MODE = true on Dashboard, then use DPAD to spin one wheel at a time.
 * Visually confirm each wheel spins in the correct direction at the correct speed.
 *   DPAD UP    — LF only
 *   DPAD DOWN  — RB only  (same diagonal as LF → should match)
 *   DPAD LEFT  — LB only
 *   DPAD RIGHT — RF only  (same diagonal as LB → should match)
 *
 * === CONTROLS ===
 * Left stick Y          — forward / back
 * Left stick X          — strafe left / right
 * Right stick X         — turn
 * DPAD (single wheel mode) — spin individual wheels
 * B                     — brake all motors (hold)
 */
@Config
@TeleOp(name = "DriveTest", group = "V2")
public class DriveTest extends OpMode {

    public static double POWER_MULTIPLIER  = 0.5;   // reduce for safer testing
    public static boolean SINGLE_WHEEL_MODE = false; // true = DPAD spins one wheel at a time

    private FtcDashboard dashboard;
    private DcMotorEx lf, lb, rf, rb;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        // Directions from Constants.java (pedroPathing)
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Left stick", "drive / strafe");
        telemetry.addData("Right stick X", "turn");
        telemetry.addData("B (hold)", "brake");
        telemetry.addData("SINGLE_WHEEL_MODE", "set on Dashboard for DPAD wheel test");
        telemetry.update();
    }

    @Override
    public void loop() {
        double lfPow, lbPow, rfPow, rbPow;

        if (gamepad1.b) {
            // Hard brake — stop all
            lfPow = lbPow = rfPow = rbPow = 0;

        } else if (SINGLE_WHEEL_MODE) {
            // Spin one wheel at a time for directional check
            double p = POWER_MULTIPLIER;
            lfPow = gamepad1.dpad_up    ? p : 0;
            rbPow = gamepad1.dpad_down  ? p : 0;
            lbPow = gamepad1.dpad_left  ? p : 0;
            rfPow = gamepad1.dpad_right ? p : 0;

        } else {
            // Normal mecanum — same formula as TeleopMoving (robot-centric)
            double forward = -gamepad1.left_stick_y  * POWER_MULTIPLIER;
            double strafe  = -gamepad1.left_stick_x  * POWER_MULTIPLIER;
            double turn    = -gamepad1.right_stick_x * POWER_MULTIPLIER;

            lfPow =  forward + strafe + turn;
            lbPow =  forward - strafe + turn;
            rfPow =  forward - strafe - turn;
            rbPow =  forward + strafe - turn;

            // Normalize if any value exceeds 1
            double max = Math.max(1.0,
                    Math.max(Math.abs(lfPow),
                    Math.max(Math.abs(lbPow),
                    Math.max(Math.abs(rfPow), Math.abs(rbPow)))));
            lfPow /= max;
            lbPow /= max;
            rfPow /= max;
            rbPow /= max;
        }

        lf.setPower(lfPow);
        lb.setPower(lbPow);
        rf.setPower(rfPow);
        rb.setPower(rbPow);

        // Per-wheel diagnostics
        double lfCurrent = lf.getCurrent(CurrentUnit.AMPS);
        double lbCurrent = lb.getCurrent(CurrentUnit.AMPS);
        double rfCurrent = rf.getCurrent(CurrentUnit.AMPS);
        double rbCurrent = rb.getCurrent(CurrentUnit.AMPS);

        // Driver hub
        telemetry.addData("LF  pwr / current", "%.2f / %.2fA", lfPow, lfCurrent);
        telemetry.addData("LB  pwr / current", "%.2f / %.2fA", lbPow, lbCurrent);
        telemetry.addData("RF  pwr / current", "%.2f / %.2fA", rfPow, rfCurrent);
        telemetry.addData("RB  pwr / current", "%.2f / %.2fA", rbPow, rbCurrent);
        telemetry.update();

        // Dashboard — graphable over time
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("LF power",       lfPow);
        packet.put("LB power",       lbPow);
        packet.put("RF power",       rfPow);
        packet.put("RB power",       rbPow);
        packet.put("LF current (A)", lfCurrent);
        packet.put("LB current (A)", lbCurrent);
        packet.put("RF current (A)", rfCurrent);
        packet.put("RB current (A)", rbCurrent);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }
}
