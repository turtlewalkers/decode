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
    public static double turretOffset = 0;
    private double hoodOffset = 0;
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 1, i = 0.1, d = 0;
    public static double pT = 0.14, iT = 0, dT = 0.00001;
    public static boolean ENABLE_FF = false;
    public static double kV = 0.0212;
    public static double kS = 0.84;
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

        RPM.add(0, 310);
        RPM.add(39.5, 310);
        RPM.add(48, 330);
        RPM.add(61, 350);
        RPM.add(90, 380);
        RPM.add(119.5, 400);
        RPM.add(136, 420);
        RPM.add(145, 440);
        RPM.add(3000, 485);
        RPM.createLUT();

        angle.add(0, 0.65);
        angle.add(39.5, 0.65);
        angle.add(48, 0.45);
        angle.add(61, 0.2);
        angle.add(90, 0.2);
        angle.add(119.5, 0.15);
        angle.add(136, 0.15);
        angle.add(145, 0.1);
        angle.add(3000, 0.1);
        angle.createLUT();

        shottime.add(0, 0.6);
        shottime.add(41.1, 0.6);
        shottime.add(51.8, 0.77);
        shottime.add(74.8, 0.72);
        shottime.add(93.3, 0.8);
        shottime.add(111, 0.85);
        shottime.add(3000, 0.85);
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
        targetAngleDeg *= turretOff;
        targetAngleDeg += turretOffset;
        targetAngleDeg = Math.max(targetAngleDeg, -100);
        targetAngleDeg = Math.min(targetAngleDeg, 270);
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        Log.d("turretPos", String.valueOf(turretPos));
        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        if (Math.abs(turretPower) <= 0.03) {
            turretPower = 0;
        }
        turret.set(turretPower / presentVoltage);
        target = RPM.get(distance);
        double theta = angle.get(distance) + hoodOffset;
        theta = Math.max(theta, 0);
        theta = Math.min(theta, 1);
        hood.set(theta);
        double vel = shooterb.getVelocity() * (2 * Math.PI / 28);
        double flywheelPID = controllerShooter.calculate(vel, target);
        flywheelPID = Math.max(-presentVoltage, Math.min(flywheelPID, presentVoltage));

        double pidVolts = flywheelPID;
        double ffvolts = kV * target;
//        ffvolts += kS * Math.signum(target);
        double flywheelVolts = pidVolts + ffvolts;
        flywheelVolts = Math.max(-presentVoltage, Math.min(flywheelVolts, presentVoltage));

        if (flywheelOn) {
            shootert.set((-1) * flywheelVolts / presentVoltage);
            shooterb.set(flywheelVolts / presentVoltage);
        } else {
            shooterb.set(0);
            shootert.set(0);
        }

    }
}