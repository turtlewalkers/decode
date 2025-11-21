package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class AutonRed extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private double redoffset = 4;

    // Poses:
    private final Pose Start = new Pose(115.5-redoffset+2.5, 128, Math.toRadians(45));

    private final Pose Paneer2 = new Pose(115-redoffset+2.5, 127.5, Math.toRadians(45));
    private final Pose ScorePositiona = new Pose(84, 85, Math.toRadians(45));
    private final Pose ScorePosition = new Pose(82, 88, Math.toRadians(315));
    private final Pose Grab1 = new Pose(96-redoffset+2.5,  85, Math.toRadians(0));
    private final Pose Collect1 = new Pose(120-redoffset, 85, Math.toRadians(0));
    private final Pose GotoGate = new Pose(119-redoffset+2.5, 74, Math.toRadians(0));
    private final Pose OpenGate = new Pose(122-redoffset, 73, Math.toRadians(0));
    private final Pose LeaveGate = new Pose(94-redoffset, 72, Math.toRadians(0));
    private final Pose Grab2 = new Pose(95-redoffset+2.5, 60, Math.toRadians(0));
    private final Pose Collect2 = new Pose(127-redoffset, 60, Math.toRadians(0));
    private final Pose Grab3 = new Pose(94-redoffset+2.5, 36, Math.toRadians(0));
    private final Pose Collect3 = new Pose(128-redoffset, 36, Math.toRadians(0));
    private final Pose Grab4Setup = new Pose(126-redoffset, 48, Math.toRadians(300));
    private final Pose Grab4 = new Pose(130-redoffset, 25, Math.toRadians(280));
    private final Pose Collect4 = new Pose(130-redoffset, 9, Math.toRadians(270));
    private final Pose byebye = new Pose(90-redoffset, 70, Math.toRadians(90));
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, ToGate, GateOpen, GateLeave, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4, tatawireless, tatawireless2;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        Paneer = new Path(new BezierLine(Start, Paneer2));
        Paneer.setLinearHeadingInterpolation(Start.getHeading(), Paneer2.getHeading());

        PreloadShoot = new Path(new BezierLine(Paneer2, ScorePositiona));
        PreloadShoot.setLinearHeadingInterpolation(Paneer2.getHeading(), ScorePositiona.getHeading());


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

        ToGate = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, GotoGate))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), GotoGate.getHeading())
                .build();

        GateOpen = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, OpenGate))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), OpenGate.getHeading())
                .build();
        GateLeave = follower.pathBuilder()
                .addPath(new BezierLine(OpenGate, LeaveGate))
                .setLinearHeadingInterpolation(OpenGate.getHeading(), LeaveGate.getHeading())
                .build();

        Goto2 = follower.pathBuilder()
                .addPath(new BezierLine(LeaveGate, Grab2))
                .setLinearHeadingInterpolation(LeaveGate.getHeading(), Grab2.getHeading())
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

        Goto4Part1 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePosition, Grab4))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab4.getHeading())
                .build();

        Goto4Part2 = follower.pathBuilder()
                .addPath(new BezierCurve(Grab4Setup, Grab4))
                .setLinearHeadingInterpolation(Grab4Setup.getHeading(), Grab4.getHeading())
                .build();

        Goto4 = follower.pathBuilder()
                .addPath(new BezierLine(Grab4, Collect4))
                .setLinearHeadingInterpolation(Grab4.getHeading(), Collect4.getHeading())
                .build();

        Shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(Collect4, ScorePosition))
                .setLinearHeadingInterpolation(Collect4.getHeading(), ScorePosition.getHeading())
                .build();


        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, () -> follower, 138, 138);

        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, Paneer),
                        intake.close(),
                        // === Preload ===
                        intake.collect(),                        // robot.intake.setPower(1);
                        shooter.flywheel(true),
                        shooter.turretOff(false),

                        new FollowPathCommand(follower, PreloadShoot),
                        intake.open(),
                        new WaitCommand(1300),

                        new FollowPathCommand(follower, Goto1, true),
                        intake.close(),


                        new FollowPathCommand(follower, Pickup1, true),

                        intake.collect(),
                        new FollowPathCommand(follower, Shoot1, true),
                        intake.open(),
                        new WaitCommand(1300),

                        new FollowPathCommand(follower, GateOpen, true).withTimeout(2000),
                        intake.close(),


                        new FollowPathCommand(follower, GateLeave, false),

                        new FollowPathCommand(follower, Goto2, true),

                        new FollowPathCommand(follower, Pickup2, true),

                        new FollowPathCommand(follower, Shoot2, true),
                        intake.open(),
                        new WaitCommand(1300),

                        new FollowPathCommand(follower, Goto3, true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup3, true),
                        intake.close(),

                        new FollowPathCommand(follower, Shoot3, true),
                        intake.open(),
                        new WaitCommand(1800),
                        intake.close(),

                        new FollowPathCommand(follower, Goto4Part1, false).withTimeout(1300),
                        intake.close(),

                        new FollowPathCommand(follower, Goto4, false).withTimeout(1000),
                        intake.close(),

                        new FollowPathCommand(follower, Shoot4, true),
                        intake.open(),
                        new WaitCommand(1800),
                        intake.close(),

                        shooter.turretOff(true),
                        new FollowPathCommand(follower, tatawireless, true),
//
                        new InstantCommand(() -> shooter.flywheel(false))
                )
        );
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
        Memory.robotPose = follower.getPose();
    }

    @Override
    public void end() {
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();
        Memory.robotPose = follower.getPose();
    }
}