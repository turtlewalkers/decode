package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import java.util.function.Supplier;

public class ShooterMove extends SubsystemBase {
    private final MotorEx shootert, shooterb, turret;
    private final ServoEx hood;
    private VoltageSensor volt;
    private final Supplier<Follower> followerSupplier;
    private boolean flywheelOn = false;
    private static double vel = 0, target = 0;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    InterpLUT shottime = new InterpLUT();
    private int turretOff = 0;
    private double turretOffset = 0;
    private double hoodOffset = 0;
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 0.6, i = 0.1, d = 0;
    public static double pT = 0.3, iT = 0, dT = 0.00001;
    public static double f = 0.0265;
    public static double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    public ShooterMove(final HardwareMap hMap, Supplier<Follower> followerSupplier, double shooterX, double shooterY, boolean turretReset) {
        this.shooterX = shooterX;
        this.shooterY = shooterY;
        this.followerSupplier = followerSupplier;
        shootert = new MotorEx(hMap, "st");
        shooterb = new MotorEx(hMap, "sb");
        turret = new MotorEx(hMap, "turret");
        hood = new ServoEx(hMap, "hood");
        volt = hMap.get(VoltageSensor.class, "Control Hub");
        shooterb.setRunMode(MotorEx.RunMode.RawPower);
        shootert.setRunMode(MotorEx.RunMode.RawPower);
        Log.d("Initial Turret Pose", String.valueOf((double)turret.getCurrentPosition() / TICKS_PER_DEGREES));
        if (turretReset) {
            turret.stopAndResetEncoder();
        }
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(MotorEx.RunMode.RawPower);

        controllerShooter = new PIDController(p, i, d);
        controllerTurret = new PIDController(pT, iT, dT);

        RPM.add(0, 330);
        RPM.add(40.5, 330);
        RPM.add(60.25, 345);
        RPM.add(90, 395);
        RPM.add(106.5, 420);
        RPM.add(132, 460);
        RPM.add(210, 485);
        RPM.add(3000, 485);
        RPM.createLUT();

        angle.add(0, 1);
        angle.add(40.5, 1);
        angle.add(60.25, 0.4);
        angle.add(90, 0.25);
        angle.add(106.5, 0.15);
        angle.add(132, 0.15);
        angle.add(210, 0.15);
        angle.add(3000, 0.15);
        angle.createLUT();

        shottime.add(0, 1);
        shottime.add(40.8, 1);
        shottime.add(61.6, 0.81);
        shottime.add(87.8, 1);
        shottime.add(106.6, 1);
        shottime.add(300, 1);
        shottime.add(3000, 1);
        shottime.createLUT();
    }

    public Command flywheel (boolean on) {
        return new InstantCommand(() -> flywheelOn = on);
    }
    public Command turretOff (boolean off) {
        return new InstantCommand(() -> turretOff = off ? 0 : 1);
    }

    public Command increaseTurretOffset () {
        return new InstantCommand(() -> turretOffset += 5);
    }

    public Command decreaseTurretOffset () {
        return new InstantCommand(() -> turretOffset -= 5);
    }

    public Command increaseHoodOffset () {
        return new InstantCommand(() -> hoodOffset += 0.05);
    }

    public Command decreaseHoodOffset () {
        return new InstantCommand(() -> hoodOffset -= 0.05);
    }

    public Command OffsetZero () {
        return new ParallelCommandGroup(
                new InstantCommand(() -> hoodOffset = 0),
                new InstantCommand(() -> turretOffset = 0)
        );
    }

    @Override
    public void periodic() {
        Pose robot = followerSupplier.get().getPose();
        double presentVoltage = volt.getVoltage();

        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();

        double dx = shooterX - robotX;
        double dy = shooterY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);

        for (int i = 0; i < 5; ++i) {
            double shotTime = shottime.get(distance);

            double vX = followerSupplier.get().getVelocity().getXComponent();
            double vY = followerSupplier.get().getVelocity().getYComponent();

            dx = shooterX - robotX - vX * shotTime;
            dy = shooterY - robotY - vY * shotTime;
            distance = Math.sqrt(dx*dx + dy*dy);
        }

        Log.d("Distance", String.valueOf(distance));
        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);
        targetAngleDeg = Math.max(targetAngleDeg, -100);
        targetAngleDeg = Math.min(targetAngleDeg, 240);
        targetAngleDeg += turretOffset;
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        Log.d("turretPos", String.valueOf(turretPos));
        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        turret.set(turretPower / presentVoltage);
        target = RPM.get(distance);
        hood.set(angle.get(distance) + hoodOffset);
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