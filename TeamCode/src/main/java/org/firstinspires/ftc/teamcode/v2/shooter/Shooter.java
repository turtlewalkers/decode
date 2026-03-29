package org.firstinspires.ftc.teamcode.v2.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDController;

/**
 * Flywheel PID tuning opmode.
 * Set target RPM (rad/s) via FTC Dashboard, tune p/i/d until velocity converges cleanly.
 * Hood angle and latch also controllable for full shot testing.
 *
 * Controls:
 *   Right trigger: intake
 *   Y (hold):      latch open
 */
@Config
@TeleOp(name = "Shooter", group = "V2")
public class Shooter extends OpMode {
    private FtcDashboard dashboard;
    private PIDController controller;

    public static double p = 0.8, i = 0.05, d = 0;
    public static double f = 0.026;
    public static double target = 0;           // rad/s — set from Dashboard
    public static double alpha = 0.6;
    public static double theta = 0;            // hood position [0, 1]
    public static boolean ENABLE_FF = true;
    public static double kV = 0.002482948;     // retune on V2
    public static double kS = 4.940223544;     // retune on V2
    public static double multiplier = 0.65;

    private DcMotorEx shooterB, shooterT, intake;
    private Servo hood, latch;
    private VoltageSensor volt;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);

        shooterB = hardwareMap.get(DcMotorEx.class, "sb");
        shooterT = hardwareMap.get(DcMotorEx.class, "st");
        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        hood     = hardwareMap.get(Servo.class, "hood");
        latch    = hardwareMap.get(Servo.class, "latch");
        volt     = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.addData("Status", "Initialized — set target RPM on Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger * multiplier);
        hood.setPosition(theta);
        latch.setPosition(gamepad1.y ? 1 : 0);

        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        double vel = shooterB.getVelocity() * (2 * Math.PI / 28);

        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));

        double ffVolts = ENABLE_FF ? (kV * target + kS * Math.signum(target)) : f * target;
        double flywheelVolts = Math.max(-presentVoltage, Math.min(pid + ffVolts, presentVoltage));

        shooterB.setPower(flywheelVolts / presentVoltage);
        shooterT.setPower((-1) * flywheelVolts / presentVoltage);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity (rad/s)", vel);
        packet.put("Target (rad/s)", target);
        packet.put("Error", target - vel);
        packet.put("PID output", pid);
        packet.put("FF volts", ffVolts);
        packet.put("Battery V", presentVoltage);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Velocity (rad/s)", vel);
        telemetry.addData("Target (rad/s)", target);
        telemetry.addData("Battery V", presentVoltage);
        telemetry.update();
    }
}
