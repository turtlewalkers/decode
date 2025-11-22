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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
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
public class AutonBlue extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    // Poses:
    private final Pose Start = new Pose(28.5-5, 135-4, Math.toRadians(135));
    private final Pose Paneer2 = new Pose(29-5, 127.5+2, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(60-5  , 85+2, Math.toRadians(135));
    private final Pose Grab1 = new Pose(48-5, 85+3, Math.toRadians(180));
    private final Pose Collect1 = new Pose(19-5, 85+3, Math.toRadians(180));
    private final Pose GotoGate = new Pose(25-5, 74+3, Math.toRadians(180));
    private final Pose OpenGate = new Pose(18-5, 73+3, Math.toRadians(180));
    private final Pose LeaveGate = new Pose(50-5, 72+3, Math.toRadians(180));
    private final Pose Grab2 = new Pose(48-5, 62+3, Math.toRadians(180));
    private final Pose Collect2 = new Pose(12-4, 62+3, Math.toRadians(180));
    private final Pose Grab3 = new Pose(48-5, 36+3, Math.toRadians(180));
    private final Pose Collect3 = new Pose(12-5, 36+3, Math.toRadians(180));
    private final Pose Grab4Setup = new Pose(16-5, 48+3, Math.toRadians(240));
    private final Pose Grab4 = new Pose(8-2, 25+3, Math.toRadians(260));
    private final Pose GotoS4 = new Pose(20-5, 40, Math.toRadians(260));
    private final Pose Collect4 = new Pose(8-2, 9+3, Math.toRadians(270));
    private final Pose byebye = new Pose(50-5, 70+3, Math.toRadians(90));
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, ToGate, GateOpen, GateLeave, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;

    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        Paneer = new Path(new BezierLine(Start, Paneer2));
        Paneer.setLinearHeadingInterpolation(Start.getHeading(), Paneer2.getHeading());

        PreloadShoot = new Path(new BezierLine(Paneer2, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Paneer2.getHeading(), ScorePosition.getHeading());


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

        Shoot4P1= follower.pathBuilder()
                .addPath(new BezierLine(Collect4, GotoS4))
                .setLinearHeadingInterpolation(Collect4.getHeading(), GotoS4.getHeading())
                .build();

        Shoot4P2= follower.pathBuilder()
                .addPath(new BezierLine(GotoS4, ScorePosition))
                .setLinearHeadingInterpolation(GotoS4.getHeading(), ScorePosition.getHeading())
                .build();



        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = false;
        Memory.autoRan = true;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, () -> follower, 6, 138, true);

        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
//                                new FollowPathCommand(follower, Paneer),
                                new FollowPathCommand(follower, PreloadShoot),
                                // === Preload ===
//                                intake.collect(),                        // robot.intake.setPower(1);
                                intake.close(),
                                shooter.flywheel(true),
                                shooter.turretOff(false)

                        ),
                        new WaitCommand(50),
                        intake.open(),
                        intake.collect(),
                        new WaitCommand(1300),
                        shooter.turretOff(true),
                        new FollowPathCommand(follower, Goto1, false),
                        intake.close(),


                        new FollowPathCommand(follower, Pickup1, true),

                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot1, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1300),
                        shooter.turretOff(true),
                        new FollowPathCommand(follower, GateOpen, true).withTimeout(2000),
                        intake.close(),
                        new WaitCommand(500),


                        new FollowPathCommand(follower, GateLeave, false),

                        new FollowPathCommand(follower, Goto2, false),

                        new FollowPathCommand(follower, Pickup2, true),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot2, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1300),
                        shooter.turretOff(true),
                        new FollowPathCommand(follower, Goto3, false),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup3, true),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot3, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1800),
                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Goto4Part1, false).withTimeout(1300),

                        new FollowPathCommand(follower, Goto4, false).withTimeout(1000),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot4P1, false, 0.8),
                        new FollowPathCommand(follower, Shoot4P2, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1800),
                        intake.close(),

                        shooter.turretOff(true),
                        new FollowPathCommand(follower, tatawireless, true),
                        new WaitCommand(1800),
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
        Memory.autoRan = true;

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}