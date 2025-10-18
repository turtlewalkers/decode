package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class ShooterTest extends OpMode {
    public static Follower follower;
    static TelemetryManager telemetryM;
    double shooterX = 135;
    double shooterY = 135;
    private Servo turret;
    private DcMotorEx flywheel;
    double multiplier = (double) 28 / 60;
    public static double RPM;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(Servo.class, "turret");
        turret.scaleRange(0, 0.67);
        turret.setDirection(Servo.Direction.FORWARD);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();
        double distance = Math.sqrt(Math.pow(Math.abs(shooterX - robotX), 2) + Math.pow(Math.abs(shooterY - robotY), 2));
        double theta = Math.atan(Math.abs(shooterX - robotX)/Math.abs(shooterY - robotY)); // radians
        double turretAngle = theta - robotHeading;
        double turretDegrees = Math.toDegrees(turretAngle);
        turretDegrees += 90;
        turretDegrees /= 180;

        turret.setPosition(1-turretDegrees);
        flywheel.setVelocity(RPM * multiplier);

        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Distance: ", distance);
        telemetry.update();
    }
}