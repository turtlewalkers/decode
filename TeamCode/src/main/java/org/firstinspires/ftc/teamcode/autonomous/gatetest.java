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
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

@Autonomous
public class gatetest extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private double redoffset = 4;

    // Poses:
    private final Pose Start = new Pose(119.5-1, 126+3, Math.toRadians(45));

    private final Pose Paneer2 = new Pose(115+2-1, 127.5-5+3, Math.toRadians(45));
    private final Pose ScorePositiona = new Pose(85, 85-4, Math.toRadians(0));
    private final Pose ScorePosition = new Pose(82+2-1, 88-5+3, Math.toRadians(315));
    private final Pose Grab1 = new Pose(96+2-1,  85-4, Math.toRadians(0));
    private final Pose Collect1 = new Pose(120+2-1, 85-4, Math.toRadians(0));
    private final Pose GotoGate = new Pose(120-1, 59+3, Math.toRadians(25));
    //    private final Pose IntakeGate = new Pose(121, 62, Math.toRadians(0));
    private final Pose CollectGate = new Pose(132, 65, Math.toRadians(28));
    private final Pose LeaveGate = new Pose(120-1, 62+3, Math.toRadians(0));
    private final Pose Grab2 = new Pose(95+2-1, 60+3, Math.toRadians(0));
    private final Pose Collect2 = new Pose(127+2-1, 56, Math.toRadians(0));
    private final Pose Grab3 = new Pose(94+2-1, 36+3, Math.toRadians(0));
    private final Pose Collect3 = new Pose(128+2-1, 36-4+3, Math.toRadians(0));
    private final Pose Grab4Setup = new Pose(126+2-1, 48-4+3, Math.toRadians(300));
    private final Pose Grab4 = new Pose(130+2-1, 25-4+3, Math.toRadians(280));
    private final Pose GotoS4 = new Pose(120-1, 28+3, Math.toRadians(280));
    private final Pose Collect4 = new Pose(130+2-1, 10+3, Math.toRadians(270));
    private final Pose byebye = new Pose(90+2-1, 70-5+3, Math.toRadians(90));
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
                        new Pose(100, 60), // Control point
                        CollectGate)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), CollectGate.getHeading())
                .build();
//        GateIntake = follower.pathBuilder()
//                .addPath(new BezierLine(IntakeGate, CollectGate))
//                .setLinearHeadingInterpolation(IntakeGate.getHeading(), CollectGate.getHeading   ())
//                .build();
        ShootGate1 = follower.pathBuilder()
                .addPath(new BezierLine(CollectGate, LeaveGate))
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
                        ScorePosition,
                        new Pose(100, 54),
                        Collect2)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect2.getHeading())
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
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(90, 28),
                        Collect3)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect3.getHeading())
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

        Shoot4P1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect4, GotoS4))
                .setLinearHeadingInterpolation(Collect4.getHeading(), GotoS4.getHeading())
                .build();

        Shoot4P2 = follower.pathBuilder()
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
        Memory.allianceRed = true;
        Memory.autoRan = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new ShooterMove(hardwareMap, () -> follower, 138, 138, true);
        intake = new Intake(hardwareMap, () -> follower, 138, 138);

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
                        /*new WaitCommand(50),
                        intake.open(),
                        intake.collect(),
                        new WaitCommand(1300),
                        shooter.turretOff(true),
                        new FollowPathCommand(follower, Goto1, false),
                        intake.close(),

                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot1, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1100),
                        shooter.turretOff(true),
                        intake.close(),


                        new FollowPathCommand(follower, Pickup2, true),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot2, true),
                        intake.open(),
                        new WaitCommand(1100),
                        shooter.turretOff(true),

                        intake.close(),*/
                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1200),
                        shooter.turretOff(false),
//                        new FollowPathCommand(follower, GateIntake, true),
                        new WaitCommand(1300),

                        new FollowPathCommand(follower, ShootGate1),
                        new FollowPathCommand(follower, ShootGate2),
                        intake.collect(),
                        intake.open(),
                        new WaitCommand(1100),

//                        new FollowPathCommand(follower, Goto3, false),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup3, true),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot3, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1100),
                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Goto4Part1, false).withTimeout(1000),

                        new FollowPathCommand(follower, Goto4, false).withTimeout(1000),
                        shooter.turretOff(false),
                        new FollowPathCommand(follower, Shoot4P1, false, 1),
                        new FollowPathCommand(follower, Shoot4P2, true),
                        new WaitCommand(50),
                        intake.open(),
                        new WaitCommand(1300),
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

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}