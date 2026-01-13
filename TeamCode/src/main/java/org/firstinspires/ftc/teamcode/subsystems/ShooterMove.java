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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ShooterMove extends SubsystemBase {
    public static final double TURRET_MIN = -90;
    public static final double TURRET_MAX = 240;   // Actual mechanical limit
    public final MotorEx shootert;
    public final MotorEx shooterb;
    public final MotorEx turret;
    private final ServoEx hood;
    private VoltageSensor volt;
    private final double TURRET_FWD_OFFSET  = -1.63; // in
    private final double TURRET_LEFT_OFFSET =  0.0;
    private final Supplier<Follower> followerSupplier;
    private boolean flywheelOn = true;
    private static double vel = 0, target = 0;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    InterpLUT shottime = new InterpLUT();
    private int turretOff = 0;
    public static double turretOffset = 0;
    private double hoodOffset = 0;
    private double shooterX, shooterY;
    private PIDController controllerShooter, controllerTurret;
    public static double p = 0.8, i = 0.05, d = 0;
    public static double pT = 2, iT = 0, dT = 0.015;
    public static boolean ENABLE_FF = false;
    public static double kV = 0.020645108; //0.002482948;
    public static double kS = 4.940223544;
    public static double f = 0.0265;
    public static double turretPos = 0;
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
    private double lastTurretTargetDeg = Double.NaN;

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
        RPM.add(42.5, 280);
        RPM.add(49.5, 300);
        RPM.add(56.5, 320);
        RPM.add(67.25, 340);
        RPM.add(77.25, 350);
        RPM.add(91.75, 370);
        RPM.add(102.75, 390);
        RPM.add(114, 415);
        RPM.add(130.75, 445);
        RPM.add(148, 484);
        RPM.add(3000, 485);
        RPM.createLUT();

        angle.add(0, 0.6);
        angle.add(42.5, 0.8);
        angle.add(49.5, 0.8);
        angle.add(56.5, 0.45);
        angle.add(67.25, 0.25);
        angle.add(77.25, 0.22);
        angle.add(91.75, 0.15);
        angle.add(102.75, 0.12);
        angle.add(114, 0.10);
        angle.add(130.75, 0.03);
        angle.add(148, 0.04);
        angle.add(3000, 0.01);
        angle.createLUT();

        shottime.add(0, 0.5);
        shottime.add(55.48460957523871, 0.59);
        shottime.add(66.36741529360351, 0.6);
        shottime.add(85.67127682322594, 0.7);
        shottime.add( 94.59729859662454, 0.72);
        shottime.add(103.13934006550497, 0.88);
        shottime.add( 116.29104823105146, 5.28-4.48);
        shottime.add(3000, 0.76);
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
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double turretX = TURRET_FWD_OFFSET * cosH - TURRET_LEFT_OFFSET * sinH;
        double turretY = TURRET_FWD_OFFSET * sinH + TURRET_LEFT_OFFSET * cosH;

        double dx = shooterX - robotX - turretX;
        double dy = shooterY - robotY - turretY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        for (int i = 0; i < 10; ++i) {
            double shotTime = shottime.get(distance);

            double vX = followerSupplier.get().getVelocity().getXComponent();
            double vY = followerSupplier.get().getVelocity().getYComponent();

            dx = shooterX - robotX - vX * shotTime - turretX;
            dy = shooterY - robotY - vY * shotTime - turretY;
            distance = Math.sqrt(dx * dx + dy * dy);
            Log.d("Distance" + i, String.valueOf(distance));
        }

        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);
        targetAngleDeg *= turretOff;
        targetAngleDeg += turretOffset;
        double[] cands = new double[] {
                targetAngleDeg,
                targetAngleDeg + 360.0,
                targetAngleDeg - 360.0
        };

        double turretPos = ((double) turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        Log.d("turretPos", String.valueOf(turretPos));

        List<Double> inRange = new ArrayList<>();
        for (double c : cands) {
            if (c >= TURRET_MIN && c <= TURRET_MAX) {
                inRange.add(c);
            }
        }

        double chosen;
        if (inRange.size() == 1) {
            chosen = inRange.get(0);
        } else if (inRange.size() == 2) {
            double d0 = Math.abs(inRange.get(0) - turretPos);
            double d1 = Math.abs(inRange.get(1) - turretPos);
            chosen = (d0 <= d1) ? inRange.get(0) : inRange.get(1);
        } else {
            double c = targetAngleDeg;
            while (c < TURRET_MIN) c += 360.0;
            while (c > TURRET_MAX) c -= 360.0;
            chosen = Math.max(TURRET_MIN, Math.min(TURRET_MAX, c));
        }

        chosen = Math.max(TURRET_MIN, Math.min(TURRET_MAX, chosen));

        double turretPower = controllerTurret.calculate(turretPos, chosen);

        if (!Limelight.turretOn) {
            turret.set(turretPower / presentVoltage);
        } else {
            turret.set(Limelight.power);
        }
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
        ffvolts += kS * Math.signum(target);
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