package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class Auton extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(60  , 85, Math.toRadians(135));
    private final Pose Grab1 = new Pose(48, 85, Math.toRadians(180));
    private final Pose Collect1 = new Pose(19, 85, Math.toRadians(180));
    private final Pose Grab2 = new Pose(48, 60, Math.toRadians(180));
    private final Pose Collect2 = new Pose(12, 60, Math.toRadians(180));
    private final Pose Grab3 = new Pose(48, 36, Math.toRadians(180));
    private final Pose Collect3 = new Pose(12, 36, Math.toRadians(180));
    private final Pose byebye = new Pose(50, 70, Math.toRadians(90));
    private final Pose byebye2 = new Pose(50, 69, Math.toRadians(90));
    private Path PreloadShoot;
    private PathChain Goto1, Pickup1, Shoot1, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, tatawireless, tatawireless2;

    public void buildpaths() {
        PreloadShoot = new Path(new BezierLine(Start, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), ScorePosition.getHeading());

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab1))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab1.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Grab1, Collect1))
                .setLinearHeadingInterpolation(Grab1.getHeading(), Collect1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .build();

        Goto2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab2))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab2.getHeading())
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(Grab2, Collect2))
                .setLinearHeadingInterpolation(Grab2.getHeading(), Collect2.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .build();

        Goto3 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab3))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab3.getHeading())
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(Grab3, Collect3))
                .setLinearHeadingInterpolation(Grab3.getHeading(), Collect3.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePosition))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePosition.getHeading())
                .build();

        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
        tatawireless2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        intake = new Intake(hardwareMap);

        buildpaths();

        SequentialCommandGroup auton = new SequentialCommandGroup(

                // === Preload ===
                intake.collect(),                        // robot.intake.setPower(1);
                // new InstantCommand(() -> updateShooterPID()), // (commented, for later)
                // new InstantCommand(() -> robot.latch.setPosition(0)), // (commented, for later)

                new FollowPathCommand(follower, PreloadShoot, true),
                // new WaitCommand(300),                 // waitShoot(300)
                // new InstantCommand(() -> runShooter()), // (commented, for later)

                // === Cycle 1 ===
                new FollowPathCommand(follower, Goto1, true),
                intake.collect(),                        // start intake
                new FollowPathCommand(follower, Pickup1, true),

                new InstantCommand(() -> {
                    telemetry.addData("Before crash", 1);
                    telemetry.update();
                }),

                new FollowPathCommand(follower, Shoot1, true),
                // new WaitCommand(300),
                // new InstantCommand(() -> runShooter()),

                // === Cycle 2 ===
                new FollowPathCommand(follower, Goto2, true),
                intake.collect(),
                new FollowPathCommand(follower, Pickup2, true),
                intake.stop(),

                new FollowPathCommand(follower, Shoot2, true),
                // new WaitCommand(300),
                // new InstantCommand(() -> runShooter()),

                // === Cycle 3 ===
                new FollowPathCommand(follower, Goto3, true),
                intake.collect(),
                new FollowPathCommand(follower, Pickup3, true),
                intake.stop(),

                new FollowPathCommand(follower, Shoot3, true),
                // new WaitCommand(300),
                // new InstantCommand(() -> runShooter()),

                // === End Pose / Park ===
                new FollowPathCommand(follower, tatawireless, true),

                new InstantCommand(() -> {
                    // Stop shooter and turret motors (commented for now)
                    // robot.shooterb.setPower(0);
                    // robot.shootert.setPower(0);
                    // robot.turret.setPower(0);

                    // Save final pose in Memory
                    Memory.robotAutoX = follower.getPose().getX();
                    Memory.robotAutoY = follower.getPose().getY();
                    Memory.robotHeading = follower.getPose().getHeading();
                })
        );

        // Schedule the autonomous sequence to run
        schedule(auton);
    }


    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();

        Memory.robotHeading = follower.getHeading();
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
    }
}