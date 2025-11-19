package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp
public class AirSort extends OpMode {
    private FtcDashboard dashboard;
    private PIDController controller;
    private TelemetryManager telemetryM;
    public static double p = 0.6, i = 0.1, d = 0;
    public static double f = 0.026;
    public static double target = 0, target2 = 0;
    private static double vel = 0;
    public static double time = 200;
    public static double alpha = 0.6;
    private Servo hood, latch;
    public static double theta = 0, theta2 = 0;

    private DcMotorEx shooterb, shootert, intake;
    private VoltageSensor volt;

    ElapsedTime timer = new ElapsedTime();
    boolean shootingSequenceActive = false;
    int shootingStep = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        hood = hardwareMap.get(Servo.class, "hood");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        latch = hardwareMap.servo.get("latch");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        timer.reset();
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger);
        hood.setPosition(theta);

        if (gamepad1.y && !shootingSequenceActive) {
            // start sequence
            shootingSequenceActive = true;
            shootingStep = 0;
            timer.reset();
            latch.setPosition(1);
            intake.setPower(1);
        }

        if (shootingSequenceActive) {
            double presentVoltage = volt.getVoltage();
            vel = shooterb.getVelocity() * (2 * Math.PI / 28);
            double pid;

            switch (shootingStep) {
                case 0:
                    // first shot
                    pid = controller.calculate(vel, target);
                    pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
                    shooterb.setPower((-1) * (pid + f * target) / presentVoltage);
                    shootert.setPower((-1) * (pid + f * target) / presentVoltage);

                    if (timer.milliseconds() > time) {
                        hood.setPosition(theta2);
                        shootingStep++;
                        timer.reset();
                    }
                    break;

                case 1:
                    // second shot
                    pid = controller.calculate(vel, target2);
                    pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
                    shooterb.setPower((-1) * (pid + f * target2) / presentVoltage);
                    shootert.setPower((-1) * (pid + f * target2) / presentVoltage);

                    if (timer.milliseconds() > time) {
                        hood.setPosition(theta);
                        shootingStep++;
                        timer.reset();
                    }
                    break;

                case 2:
                    // third shot
                    pid = controller.calculate(vel, target);
                    pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
                    shooterb.setPower((-1) * (pid + f * target) / presentVoltage);
                    shootert.setPower((-1) * (pid + f * target) / presentVoltage);

                    shootingSequenceActive = false; // done
                    latch.setPosition(0);
                    break;
            }
        }

        // telemetry
        double presentVoltage = volt.getVoltage();
        vel = shooterb.getVelocity() * (2 * Math.PI / 28);
        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        shooterb.setPower((-1) * (pid + f * target) / presentVoltage);
        shootert.setPower((-1) * (pid + f * target) / presentVoltage);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity", vel);
        packet.put("Target", target);
        packet.put("Power", pid);
        dashboard.sendTelemetryPacket(packet);
    }

}