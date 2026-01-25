package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

@Config
@TeleOp
public class Turret extends OpMode {
    private FtcDashboard dashboard;

    private ProfiledPIDController controller;
    public static double p = 1.68, i = 0, d = 0.03;
    public static double maxV = 530, maxA = 50000;
    public static TrapezoidProfile.Constraints trapezoidProfileConstraints = new TrapezoidProfile.Constraints(530,0);
    public static double target = 0;
    public static double acceleration = 0;
    public static double currentVelocity = 0, previousVelocity = 0;
    private VoltageSensor volt;
    private ElapsedTime timer = new ElapsedTime();
    private DcMotorEx turret;
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    @Override
    public void init() {
        controller = new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(maxV, maxA));
//        controller = new PIDController(p, i, d);
        dashboard = FtcDashboard.getInstance();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        timer.reset();
    }

    @Override
    public void loop() {
        double presentVoltage = volt.getVoltage();
        controller.setPID(p, i, d);
        controller.setConstraints(new TrapezoidProfile.Constraints(maxV, maxA));
        double pos = turret.getCurrentPosition() / TICKS_PER_DEGREES;
        double pid = controller.calculate(pos, target);
        turret.setPower(pid / presentVoltage);

        currentVelocity = turret.getVelocity() / TICKS_PER_DEGREES;
        acceleration = (currentVelocity - previousVelocity) / timer.seconds();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Position", pos);
        packet.put("Target", target);
        packet.put("Power", pid);
        packet.put("Velocity", currentVelocity);
        packet.put("Acceleration", acceleration);
        dashboard.sendTelemetryPacket(packet);

        previousVelocity = currentVelocity;
        timer.reset();
    }
}