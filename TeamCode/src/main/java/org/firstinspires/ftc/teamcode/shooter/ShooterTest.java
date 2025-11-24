package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * ShooterFFTest
 *
 * Purpose:
 *  - Manually command shooter motor power.
 *  - Log battery voltage, power, and shooter speed (RPM / rad/s).
 *  - Use data to estimate kV and kS for feedforward.
 *
 * Controls:
 *   - DPAD_UP:    increase testPower by +0.02
 *   - DPAD_DOWN:  decrease testPower by -0.02
 *   - X:          reset testPower to 0
 *   - A:          toggle RUN/STOP (when STOP, power = 0)
 *
 * Usage:
 *   1. Start with testPower = 0, RUN=false.
 *   2. Press A to enable RUN.
 *   3. Slowly tap DPAD_UP to increase power until shooter just starts spinning.
 *      -> This point gives you kS ≈ power * batteryVoltage.
 *   4. Continue increasing power in steps (0.2, 0.3, 0.4, 0.5, 0.6),
 *      wait ~1 second at each step, record:
 *         - batteryVoltage
 *         - testPower
 *         - shooterRPM or rad/s
 *      in a spreadsheet for kV fitting.
 */

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    // Dashboard-tunable starting power
    public static double testPower = 0.0;   // [-1.0, 1.0]
    public static boolean runShooter = false;

    // Motor names (adjust if yours differ)
    public static String BOTTOM_MOTOR_NAME = "sb";
    public static String TOP_MOTOR_NAME = "st";

    // Encoder constants
    public static double TICKS_PER_REV = 28.0;

    // Logging tag
    private static final String TAG = "ShooterFFTest";

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware init
        MotorEx shooterBottom = new MotorEx(hardwareMap, BOTTOM_MOTOR_NAME);
        MotorEx shooterTop    = new MotorEx(hardwareMap, TOP_MOTOR_NAME);

        shooterBottom.setRunMode(MotorEx.RunMode.RawPower);
        shooterTop.setRunMode(MotorEx.RunMode.RawPower);

        VoltageSensor volt = hardwareMap.get(VoltageSensor.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("ShooterFFTest ready.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A: toggle RUN/STOP");
        telemetry.addLine("  DPAD_UP:   +0.02 power");
        telemetry.addLine("  DPAD_DOWN: -0.02 power");
        telemetry.addLine("  X: reset power to 0");
        telemetry.addLine("");
        telemetry.addLine("Use this to find kS (first move) and kV (fit Volts vs speed).");
        telemetry.update();

        boolean lastA = false;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastX = false;

        waitForStart();

        while (opModeIsActive()) {
            double batteryV = volt.getVoltage();

            // --- Edge-detect buttons for clean steps ---

            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean x = gamepad1.x;

            // Toggle runShooter on A press
            if (a && !lastA) {
                runShooter = !runShooter;
            }

            // Increase power on DPAD_UP press
            if (up && !lastUp) {
                testPower += 0.02;
            }

            // Decrease power on DPAD_DOWN press
            if (down && !lastDown) {
                testPower -= 0.02;
            }

            // Reset power on X press
            if (x && !lastX) {
                testPower = 0.0;
            }

            // Save previous states
            lastA = a;
            lastUp = up;
            lastDown = down;
            lastX = x;

            // Clamp power
            if (testPower > 1.0) testPower = 1.0;
            if (testPower < -1.0) testPower = -1.0;

            double appliedPower = runShooter ? testPower : 0.0;

            // Apply to motors (both same direction for this test)
            shooterBottom.set(-appliedPower);
            shooterTop.set(-appliedPower);

            // Read velocity from bottom motor
            // getVelocity() -> ticks per second (MotorEx)
            double ticksPerSec = shooterBottom.getVelocity();
            double revPerSec   = ticksPerSec / TICKS_PER_REV;
            double rpm         = revPerSec * 60.0;
            double radPerSec   = revPerSec * 2.0 * Math.PI;

            // Log to Driver Station
            telemetry.addData("RUN", runShooter);
            telemetry.addData("testPower", "%.3f", testPower);
            telemetry.addData("appliedPower", "%.3f", appliedPower);
            telemetry.addData("batteryV", "%.2f", batteryV);
            telemetry.addData("ticks/s", "%.1f", ticksPerSec);
            telemetry.addData("RPM", "%.1f", rpm);
            telemetry.addData("rad/s", "%.1f", radPerSec);
            telemetry.addLine("NOTE: For kS, find smallest power where RPM > 0.");
            telemetry.update();

            // Log to Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("RUN", runShooter);
            packet.put("testPower", testPower);
            packet.put("appliedPower", appliedPower);
            packet.put("batteryV", batteryV);
            packet.put("ticksPerSec", ticksPerSec);
            packet.put("RPM", rpm);
            packet.put("radPerSec", radPerSec);
            dashboard.sendTelemetryPacket(packet);

            // Optional: log to RobotLog for later adb logcat dump
            RobotLog.dd(TAG,
                    "RUN=%b, power=%.3f, battery=%.2f, rpm=%.1f, rad_s=%.1f",
                    runShooter, appliedPower, batteryV, rpm, radPerSec);

            // Small delay – 20ms loop
            sleep(20);
        }

        // Stop motors when OpMode ends
        shooterBottom.set(0.0);
        shooterTop.set(0.0);
    }
}

