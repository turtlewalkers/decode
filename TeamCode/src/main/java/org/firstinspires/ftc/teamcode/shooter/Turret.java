package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

@Config
@TeleOp
public class Turret extends OpMode {
    private FtcDashboard dashboard;

    private PIDController controller;

    public static double p = 0.12, i = 0, d = 0;
    public static double target = 0;

    private final double ticks = 537.7 / 3;

    private DcMotorEx turret;
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        dashboard = FtcDashboard.getInstance();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double pos = turret.getCurrentPosition() / TICKS_PER_DEGREES;
        double pid = controller.calculate(pos, target);
        turret.setPower(pid);

        telemetry.addData("Position", pos);
        telemetry.addData("Target", target);
        telemetry.addData("Power", pid);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", pos);
        packet.put("Target", target);
        packet.put("Power", pid);
        dashboard.sendTelemetryPacket(packet);
    }
}