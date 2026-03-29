package org.firstinspires.ftc.teamcode.v2.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Flywheel FF characterization opmode.
 *
 * Purpose:
 *   Sweep motor power, record battery voltage and shooter velocity at each step.
 *   Use the data to fit kV and kS for feedforward in Shooter.java / ShooterMove.java.
 *
 * Procedure:
 *   1. Press A to enable RUN.
 *   2. Tap DPAD_UP slowly until RPM just starts moving → kS ≈ testPower × batteryV
 *   3. Increase power in steps (0.2, 0.3, 0.4, 0.5, 0.6), wait ~1s at each step.
 *   4. Record (batteryV × testPower) vs rad/s in a spreadsheet → slope = kV, intercept = kS.
 *
 * Controls:
 *   A:          toggle RUN / STOP
 *   DPAD_UP:    +0.02 power
 *   DPAD_DOWN:  -0.02 power
 *   X:          reset power to 0
 */
@Config
@TeleOp(name = "ShooterTest", group = "V2")
public class ShooterTest extends LinearOpMode {

    public static double testPower = 0.0;
    public static boolean runShooter = false;
    public static double TICKS_PER_REV = 28.0;

    private static final String TAG = "ShooterFFTest";

    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx shooterB = new MotorEx(hardwareMap, "sb");
        MotorEx shooterT = new MotorEx(hardwareMap, "st");
        shooterB.setRunMode(MotorEx.RunMode.RawPower);
        shooterT.setRunMode(MotorEx.RunMode.RawPower);

        VoltageSensor volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("ShooterFFTest ready. Controls:");
        telemetry.addLine("  A: toggle RUN/STOP");
        telemetry.addLine("  DPAD_UP: +0.02  DPAD_DOWN: -0.02  X: reset");
        telemetry.update();

        boolean lastA = false, lastUp = false, lastDown = false, lastX = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean a    = gamepad1.a;
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean x    = gamepad1.x;

            if (a    && !lastA)    runShooter = !runShooter;
            if (up   && !lastUp)   testPower  = Math.min(1.0,  testPower + 0.02);
            if (down && !lastDown) testPower  = Math.max(-1.0, testPower - 0.02);
            if (x    && !lastX)    testPower  = 0.0;

            lastA = a; lastUp = up; lastDown = down; lastX = x;

            double appliedPower = runShooter ? testPower : 0.0;
            shooterB.set(-appliedPower);
            shooterT.set(appliedPower);

            double batteryV  = volt.getVoltage();
            double ticksPerSec = shooterB.getVelocity();
            double rpm       = ticksPerSec / TICKS_PER_REV * 60.0;
            double radPerSec = ticksPerSec / TICKS_PER_REV * 2.0 * Math.PI;

            double sbcurrentAmps = shooterB.getCurrent(CurrentUnit.AMPS);
            double stcurrentAmps = shooterT.getCurrent(CurrentUnit.AMPS);

            telemetry.addData("RUN",          runShooter);
            telemetry.addData("testPower",    "%.3f", testPower);
            telemetry.addData("batteryV",     "%.2f", batteryV);
            telemetry.addData("RPM",          "%.1f", rpm);
            telemetry.addData("rad/s",        "%.1f", radPerSec);
            telemetry.addData("Volts applied","%.3f", appliedPower * batteryV);
            telemetry.addLine("kS ~ power*V where RPM first moves");
            telemetry.addLine("kV ~ slope of (Volts applied) vs (rad/s)");
            telemetry.addData("sb Motor Current", sbcurrentAmps);
            telemetry.addData("st Motor Current", stcurrentAmps);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("RUN", runShooter);
            packet.put("testPower", testPower);
            packet.put("batteryV", batteryV);
            packet.put("RPM", rpm);
            packet.put("radPerSec", radPerSec);
            packet.put("voltsApplied", appliedPower * batteryV);
            dashboard.sendTelemetryPacket(packet);

            RobotLog.dd(TAG, "RUN=%b power=%.3f battery=%.2f rpm=%.1f rad_s=%.1f",
                    runShooter, appliedPower, batteryV, rpm, radPerSec);

            sleep(20);
        }

        shooterB.set(0.0);
        shooterT.set(0.0);
    }
}
