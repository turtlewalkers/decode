/*
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
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final MotorEx shootert, shooterb, turret;
    private final ServoEx hood;
    private VoltageSensor volt;
    private Intake intake;
    private final double TURRET_FWD_OFFSET = -1.63; // in
    private final double TURRET_LEFT_OFFSET = 0.0;
    private final Supplier<Follower> followerSupplier;
    private boolean flywheelOn = false;
    private static double vel = 0, target = 0;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    InterpLUT shottime = new InterpLUT();
    private int turretOff = 0;
    public static double turretOffset = 0;
    private static final double HOOD_UP = 0.5;
    private static final double HOOD_STRAIGHT = 0.2;
    private static final int STAGGER_TIME = 50;
    private boolean manualHoodControl = false;
    private double hoodOffset = 0;
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 1, i = 0.1, d = 0;
    public static double pT = 0.5, iT = 0, dT = 0.015;
    public static boolean ENABLE_FF = false;
    public static double kV = 0.0212;
    public static double kS = 0.84;
    public static double f = 0.0265;
    public static double TICKS_PER_DEGREES = ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0 * 3.0) / 360.0;

    public Shooter(final HardwareMap hMap, Supplier<Follower> followerSupplier, double shooterX, double shooterY, boolean turretReset) {
        this.shooterX = shooterX;
        this.shooterY = shooterY;
        this.followerSupplier = followerSupplier;
        intake = new Intake(hMap, followerSupplier, shooterX, shooterY);
        shootert = new MotorEx(hMap, "st");
        shooterb = new MotorEx(hMap, "sb");
        turret = new MotorEx(hMap, "turret");
        hood = new ServoEx(hMap, "hood");
        volt = hMap.get(VoltageSensor.class, "Control Hub");
        shooterb.setRunMode(MotorEx.RunMode.RawPower);
        shootert.setRunMode(MotorEx.RunMode.RawPower);
        Log.d("Initial Turret Pose", String.valueOf((double) turret.getCurrentPosition() / TICKS_PER_DEGREES));
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
        RPM.add(61, 340);
        RPM.add(90, 390);
        RPM.add(119.5, 440);
        RPM.add(136, 420);
        RPM.add(145, 440);
        RPM.add(3000, 485);
        RPM.createLUT();

        angle.add(0, 0.6);
        angle.add(39.5, 0.6);
        angle.add(48, 0.45);
        angle.add(61, 0.3);
        angle.add(90, 0.19);
        angle.add(119.5, 0.19);
        angle.add(136, 0.15);
        angle.add(145, 0.12);
        angle.add(3000, 0.1);
        angle.createLUT();
    }

    public Command flywheel(boolean on) {
        return new InstantCommand(() -> flywheelOn = on);
    }

    public Command turretOff(boolean off) {
        return new InstantCommand(() -> turretOff = off ? 0 : 1);
    }

    public Command increaseTurretOffset() {
        return new InstantCommand(() -> turretOffset += 5);
    }

    public Command decreaseTurretOffset() {
        return new InstantCommand(() -> turretOffset -= 5);
    }

    public Command increaseHoodOffset() {
        return new InstantCommand(() -> hoodOffset += 0.05);
    }

    public Command decreaseHoodOffset() {
        return new InstantCommand(() -> hoodOffset -= 0.05);
    }

    public Command OffsetZero() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> hoodOffset = 0),
                new InstantCommand(() -> turretOffset = 0)
        );
    }

    public Command shootUp() {
        return new InstantCommand(() -> {
            manualHoodControl = true;
            hood.set(HOOD_UP);
        });
    }

    public Command shootStraight() {
        return new InstantCommand(() -> {
            manualHoodControl = true;
            hood.set(HOOD_STRAIGHT);
        });
    }

    public Command resumeAutoHood() {
        return new InstantCommand(() -> manualHoodControl = false);
    }

    public Command stagger() {
        return new WaitCommand(STAGGER_TIME);
    }

    // add this somewhere near your other commands (replaces the existing airsort method)
    public Command airsort(String from, String to) {
        from = from.toUpperCase();
        to = to.toUpperCase();

        Command seq;

        if (from.equals("GPP") && to.equals("PPG")) {
            // G up, P straight, P straight (kinda favorable)
            seq = new SequentialCommandGroup(
                    new WaitCommand(400),
                    shootUp(),
                    shootStraight(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else if (from.equals("GPP") && to.equals("PGP")) {
            // Favorable: G up, P straight, stagger, P straight
            seq = new SequentialCommandGroup(
                    new WaitCommand(400),
                    shootUp(),
                    shootStraight(),
                    stagger(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else if (from.equals("PGP") && to.equals("GPP")) {
            // Favorable (stagger): P up, G straight, stagger, P straight
            seq = new SequentialCommandGroup(
                    new WaitCommand(400),
                    shootUp(),
                    shootStraight(),
                    stagger(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else if (from.equals("PGP") && to.equals("PPG")) {
            // Favorable (stagger): P up, stagger, G up, P straight
            seq = new SequentialCommandGroup(
                    new WaitCommand(400),
                    shootUp(),
                    stagger(),
                    shootUp(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else if (from.equals("PPG") && to.equals("GPP")) {
            // P up, P up, G straight (kinda favorable)
            seq = new SequentialCommandGroup(
                    new WaitCommand(400),
                    shootUp(),
                    shootUp(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else if (from.equals("PPG") && to.equals("PGP")) {
            // Favorable variant: P up, stagger, P up, G straight
            seq = new SequentialCommandGroup(
                    new WaitCommand(0),
                    shootUp(),
                    stagger(),
                    new WaitCommand(400),
                    shootUp(),
                    shootStraight(),
                    resumeAutoHood()
            );
        } else {
            String finalFrom = from;
            String finalTo = to;
            seq = new InstantCommand(() ->
                    Log.e("AIRSORT", "Invalid airsort mapping: " + finalFrom + " -> " + finalTo)
            );
        }

        // While the airsort sequence runs, keep intake open and collecting.
        // RunCommand will be interrupted automatically when the seq finishes.
        Command intakeRun = new RunCommand(() -> {
            intake.open();
            intake.collect();
        }, intake);

        return new ParallelCommandGroup(intakeRun, seq);
    }

    @Override
    public void periodic() {
        Pose robot = followerSupplier.get().getPose();
        double presentVoltage = volt.getVoltage();

        double robotX = robot.getX();
        double robotY = robot.getY();
        double robotHeading = robot.getHeading();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double turretX = TURRET_FWD_OFFSET * cosH - TURRET_LEFT_OFFSET * sinH;
        double turretY = TURRET_FWD_OFFSET * sinH + TURRET_LEFT_OFFSET * cosH;

        double dx = shooterX - robotX;
        double dy = shooterY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);

        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);
        targetAngleDeg *= turretOff;
        targetAngleDeg += turretOffset;
        targetAngleDeg = Math.max(targetAngleDeg, -130);
        targetAngleDeg = Math.min(targetAngleDeg, 260);
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        Log.d("turretPos", String.valueOf(turretPos));
        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        turret.set(turretPower / presentVoltage);
        target = RPM.get(distance);
        double theta = angle.get(distance) + hoodOffset;
        theta = Math.max(theta, 0);
        theta = Math.min(theta, 1);
        if (!manualHoodControl) {
            hood.set(theta);
        } else {
            intake.collect();
            intake.open();
        }
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
}*/
