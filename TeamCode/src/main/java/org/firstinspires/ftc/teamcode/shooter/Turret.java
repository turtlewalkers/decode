package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@TeleOp
public class Turret extends OpMode {
    private PIDController controller;

    public static double p = -0.025, i = 0, d = 0;
    public static double target = 0;

    private final double ticks = 537.7 / 3;

    private DcMotorEx turret;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int pos = turret.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        turret.setPower(pid);

        telemetry.addData("Position", pos);
        telemetry.addData("Target", target);
        telemetry.addData("Power", pid);
        telemetry.update();
    }
}