package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

@Autonomous
public class Red15Friendly extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private double redoffset = 4;

    // Poses:
    private final Pose Start = new Pose(117, 128, Math.toRadians(45));

    private final Pose Paneer2 = new Pose(115+2-1, 121.5, Math.toRadians(45));
    private final Pose ScorePositiona = new Pose(85, 84, Math.toRadians(0));
    private final Pose ScorePosition = new Pose(84, 84, Math.toRadians(315));
    private final Pose Collect1 = new Pose(124, 84, Math.toRadians(0));
    private final Pose CollectGate = new Pose(124, 67, Math.toRadians(28));
    private final Pose LeaveGate = new Pose(129, 62, Math.toRadians(28));
    private final Pose Collect2 = new Pose(130, 60, Math.toRadians(0));
    private final Pose Collect3 = new Pose(130, 36, Math.toRadians(0));
    private final Pose Grab4Setup = new Pose(126+2-1, 48-4-3, Math.toRadians(300));
    private final Pose Grab4 = new Pose(130+2-1, 25-4-3, Math.toRadians(280));
    private final Pose GotoS4 = new Pose(120-1, 28-3, Math.toRadians(280));
    private final Pose Collect4 = new Pose(138, 9, Math.toRadians(270));
    private final Pose byebye = new Pose(115, 72, Math.toRadians(270));
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, ToGate, GotoIntakeGate, GateIntake, ShootGate1, ShootGate2, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        Paneer = new Path(new BezierLine(Start, Paneer2));
        Paneer.setLinearHeadingInterpolation(Start.getHeading(), Paneer2.getHeading());

        PreloadShoot = new Path(new BezierLine(Paneer2, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Paneer2.getHeading(), ScorePosition.getHeading());


        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePositiona, Collect1))
                .setLinearHeadingInterpolation(ScorePositiona.getHeading(), Collect1.getHeading())
                .build();

//        Pickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(Grab1, Collect1))
//                .setLinearHeadingInterpolation(Grab1.getHeading(), Collect1.getHeading())
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .build();

//        ToGate = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePosition, GotoGate))
//                .setLinearHeadingInterpolation(ScorePosition.getHeading(), GotoGate.getHeading())
//                .build();
        GotoIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(100, 64), // Control point
                        CollectGate)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), CollectGate.getHeading())
                .build();
//        GateIntake = follower.pathBuilder()
//                .addPath(new BezierLine(IntakeGate, CollectGate))
//                .setLinearHeadingInterpolation(IntakeGate.getHeading(), CollectGate.getHeading   ())
//                .build();
        ShootGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        CollectGate,
                        new Pose(130, 63),
                        LeaveGate)
                )
                .setLinearHeadingInterpolation(CollectGate.getHeading(), LeaveGate.getHeading())
                .build();
        ShootGate2 = follower.pathBuilder()
                .addPath(new BezierLine(LeaveGate, ScorePosition))
                .setLinearHeadingInterpolation(LeaveGate.getHeading(), ScorePosition.getHeading())
                .build();

//        Goto2 = follower.pathBuilder()
//                .addPath(new BezierLine(ScorePosition, Grab2))
//                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab2.getHeading())
//                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePositiona,
                        new Pose(100, 57.5),
                        Collect2)
                )
                .setLinearHeadingInterpolation(ScorePositiona.getHeading(), Collect2.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(90, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect3.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePositiona))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePositiona.getHeading())
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
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(132, 70), // Control point
                        Collect4)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect4.getHeading())
                .build();

        Shoot4P1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect4, ScorePosition))
                .setLinearHeadingInterpolation(Collect4.getHeading(), ScorePosition.getHeading())
                .build();
//
//        Shoot4P2 = follower.pathBuilder()
//                .addPath(new BezierLine(GotoS4, ScorePosition))
//                .setLinearHeadingInterpolation(GotoS4.getHeading(), ScorePosition.getHeading())
//                .build();


        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = true;
        Memory.autoRan = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new Shooter(hardwareMap, () -> follower, 138, 138, true);
        intake = new Intake(hardwareMap, () -> follower, 138, 138);

        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, PreloadShoot),
                                intake.close(),
                                shooter.flywheel(true),
                                shooter.turretOff(false)

                        ),
                        intake.open(),
                        new WaitCommand(70),
                        intake.collect(),
                        new WaitCommand(1400),
                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup2, false),
                        shooter.turretOff(false),


                        new FollowPathCommand(follower, Shoot2, true),
                        intake.open(),
                        new WaitCommand(1400),
                        shooter.turretOff(true),
                        intake.close(),

                        intake.close(),
                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1200),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, ShootGate1),
                        new WaitCommand(1300),
                        intake.stop(),
                        intake.open(),
                        new FollowPathCommand(follower, ShootGate2),
                        intake.collect(),
                        new WaitCommand(1400),

//                        new FollowPathCommand(follower, Goto3, false),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup3, true),
                        shooter.turretOff(false),
                        intake.stop(),
                        intake.open(),
                        new FollowPathCommand(follower, Shoot3, true),
                        intake.collect(),
                        new WaitCommand(1400),
                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Goto1, true),
                        shooter.turretOff(false),
                        intake.stop(),
                        intake.open(),
                        new FollowPathCommand(follower, Shoot1, true),
                        intake.collect(),
                        new WaitCommand(1400),
                        shooter.turretOff(true),
                        intake.close(),

                        shooter.turretOff(true),
                        new FollowPathCommand(follower, tatawireless, true),
                        new WaitCommand(500),
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
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}