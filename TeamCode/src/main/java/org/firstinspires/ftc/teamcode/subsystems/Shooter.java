package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    public static double x = 0;
    private final MotorEx shootert, shooterb, turret;
    private final ServoEx hood;
    private VoltageSensor volt;
    private final Supplier<Pose> poseSupplier;
    private boolean flywheelOn = false;
    private static double vel = 0, target = 0;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 0.6, i = 0.1, d = 0;
    public static double pT = 0.3, iT = 0, dT = 0;
    public static double f = 0.0265;
    public static double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    public Shooter(final HardwareMap hMap, Supplier<Pose> poseSupplier, double shooterX, double shooterY) {
        this.shooterX = shooterX;
        this.shooterY = shooterY;
        this.poseSupplier = poseSupplier;
        shootert = new MotorEx(hMap, "st");
        shooterb = new MotorEx(hMap, "sb");
        turret = new MotorEx(hMap, "turret");
        hood = new ServoEx(hMap, "hood");
        volt = hMap.get(VoltageSensor.class, "Control Hub");
        shooterb.setRunMode(MotorEx.RunMode.RawPower);
        shootert.setRunMode(MotorEx.RunMode.RawPower);
        turret.setRunMode(MotorEx.RunMode.RawPower);

        controllerShooter = new PIDController(p, i, d);
        controllerTurret = new PIDController(pT, iT, dT);

        RPM.add(0, 315);
        RPM.add(48.5, 315);
        RPM.add(68.25, 330);
        RPM.add(98, 380);
        RPM.add(114.5, 410);
        RPM.add(210, 410);
        RPM.createLUT();

        angle.add(0, 1);
        angle.add(48.5, 1);
        angle.add(68.25, 0.3);
        angle.add(98, 0.15);
        angle.add(114.5, 0.05);
        angle.add(210, 0.05);
        angle.createLUT();
    }

    public Command flywheel (boolean on) {
        return new InstantCommand(() -> flywheelOn = on);
    }

    @Override
    public void periodic() {
        Pose robot = poseSupplier.get();
        double presentVoltage = volt.getVoltage();

        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();

        double dx = shooterX - robotX;
        double dy = shooterY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);
        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading) - x;
        Log.d("Target angle 2", String.valueOf(targetAngleDeg));
        targetAngleDeg = Math.max(targetAngleDeg, -100);
        targetAngleDeg = Math.min(targetAngleDeg, 240);
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        Log.d("turretPos", String.valueOf(turretPos));
        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        turret.set(turretPower / presentVoltage);
        target = RPM.get(distance);
        hood.set(angle.get(distance));
        double vel = shooterb.getVelocity() * (2 * Math.PI / 28);
        double flywheelPID = controllerShooter.calculate(vel, target);
        flywheelPID = Math.max(-presentVoltage, Math.min(flywheelPID, presentVoltage));

        if (flywheelOn) {
            shootert.set((-1) * flywheelPID / presentVoltage);
            shooterb.set((-1) * flywheelPID / presentVoltage);
        } else {
            shooterb.set(0);
            shootert.set(0);
        }

    }
}