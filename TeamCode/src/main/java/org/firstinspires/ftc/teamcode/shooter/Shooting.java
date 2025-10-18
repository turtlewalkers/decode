package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Shooting extends OpMode {
    public static Follower follower;
    static TelemetryManager telemetryM;
    double shooterX = 135;
    double shooterY = 135;
    private DcMotorEx turret;

    private PIDController controller;

    public static double p = -0.025, i = 0, d = 0;
    public static double target = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
//        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
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
        controller = new PIDController(p, i, d);
    }

    /**
     * This updates the robot's pose estimate, the simple bnbnmecanum drive, and updates the
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
        double theta = Math.atan(Math.abs(shooterY - robotY) / Math.abs(shooterX - robotX)); // radians
        double turretAngle = theta - robotHeading;
        double turretDegrees = Math.toDegrees(turretAngle);
        turretDegrees = (turretDegrees + 180) % 360 / 360.0;
        target = turretDegrees * 1610;
        controller.setPID(p, i, d);
        int pos = turret.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        turret.setPower(pid);
//        flywheel.setVelocity(rpm * multiplier);

        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Distance: ", robotX);
        telemetry.addData("Distance: ", robotY);

        telemetry.update();
    }
}