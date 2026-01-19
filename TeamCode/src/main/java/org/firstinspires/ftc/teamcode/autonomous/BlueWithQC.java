package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
public class BlueWithQC extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(144-117, 128, Math.toRadians(135));

    private final Pose Paneer2 = new Pose(144-115+2-1, 121.5, Math.toRadians(135));
    private final Pose ScorePositionA = new Pose(144-88, 82, Math.toRadians(180));
    private final Pose ScorePositionB = new Pose(144-88, 82, Math.toRadians(220));
    private final Pose ScorePositionC = new Pose(60, 75.4, Math.toRadians(212));
    private final Pose ScorePositionD = new Pose(144-88, 78, Math.toRadians(245));
    private final Pose Collect1 = new Pose(144-123, 82, Math.toRadians(180));
    private final Pose CollectGate = new Pose(16.3, 62.6, Math.toRadians(180-19));
    private final Pose LeaveGate = new Pose(16, 56, Math.toRadians(180-23));
    private final Pose Collect2 = new Pose(22, 60, Math.toRadians(180));
    private final Pose OpenGateAfter2 = new Pose(20, 70, Math.toRadians(180));
    private final Pose Collect3 = new Pose(144-126, 36, Math.toRadians(180));
    private final Pose Grab4Setup = new Pose(144-(126+2-1), 48-4-3, Math.toRadians(240));
    private final Pose Grab4 = new Pose(144-(130+2-1), 25-4-3, Math.toRadians(260));
    private final Pose GotoS4 = new Pose(144-(120-1), 28-3, Math.toRadians(260));
    private final Pose Collect4 = new Pose(6, 9, Math.toRadians(270));
    private final Pose byebye = new Pose(52, 76, Math.toRadians(225));
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, ToGate, GotoIntakeGate, GateIntake, ShootGate1, ShootGate2, OpenGateFor2, ShootGate3, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;
    private GoBildaPinpointDriver pinpoint;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        Paneer = new Path(new BezierLine(Start, Paneer2));
        Paneer.setLinearHeadingInterpolation(Start.getHeading(), Paneer2.getHeading());
        Paneer.setTimeoutConstraint(50);

        PreloadShoot = new Path(new BezierLine(Paneer2, ScorePositionA));
        PreloadShoot.setLinearHeadingInterpolation(Paneer2.getHeading(), ScorePositionA.getHeading());
        PreloadShoot.setTimeoutConstraint(50);

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePositionA, Collect1))
                .setLinearHeadingInterpolation(ScorePositionA.getHeading(), Collect1.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePositionB))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePositionB.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GotoIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePositionC,
                        new Pose(36, 59.5),
                        CollectGate)
                )
                .setLinearHeadingInterpolation(ScorePositionC.getHeading(), CollectGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        ShootGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        CollectGate,
                        new Pose(17, 58),
                        LeaveGate)
                )
                .setLinearHeadingInterpolation(CollectGate.getHeading(), LeaveGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        ShootGate2 = follower.pathBuilder()
                .addPath(new BezierLine(LeaveGate, ScorePositionC))
                .setLinearHeadingInterpolation(LeaveGate.getHeading(), ScorePositionC.getHeading())
                .setTimeoutConstraint(50)
                .build();

        ShootGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        CollectGate,
                        new Pose(18.3, 60.31578947368421),
                        new Pose(21.08947368421052, 62.013157894736835),
                        ScorePositionC)
                )
                .setLinearHeadingInterpolation(CollectGate.getHeading(), ScorePositionC.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePositionC,
                        new Pose(54, 59.5),
                        Collect2)
                )
                .setLinearHeadingInterpolation(ScorePositionC.getHeading(), Collect2.getHeading())
                .setTimeoutConstraint(50)
                .build();

        OpenGateFor2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Collect2,
                        new Pose(30, 65),
                        OpenGateAfter2)
                )
                .setLinearHeadingInterpolation(Collect2.getHeading(), OpenGateAfter2.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(OpenGateAfter2, ScorePositionC))
                .setLinearHeadingInterpolation(OpenGateAfter2.getHeading(), ScorePositionC.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePositionC,
                        new Pose(57, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(ScorePositionC.getHeading(), Collect3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePositionD))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePositionD.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4Part1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePositionA, Grab4))
                .setLinearHeadingInterpolation(ScorePositionD.getHeading(), Grab4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4Part2 = follower.pathBuilder()
                .addPath(new BezierLine(Grab4Setup, Grab4))
                .setLinearHeadingInterpolation(Grab4Setup.getHeading(), Grab4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePositionA,
                        new Pose(132, 70),
                        Collect4)
                )
                .setLinearHeadingInterpolation(ScorePositionA.getHeading(), Collect4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot4P1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect4, ScorePositionA))
                .setLinearHeadingInterpolation(Collect4.getHeading(), ScorePositionA.getHeading())
                .setTimeoutConstraint(50)
                .build();

        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePositionA, byebye))
                .setLinearHeadingInterpolation(ScorePositionD.getHeading(), byebye.getHeading())
                .setTimeoutConstraint(50)
                .build();
    }

    @Override
    public void reset() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = false;
        Memory.autoRan = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new ShooterMove(hardwareMap, () -> follower, 6, 138, true);
        intake = new Intake(hardwareMap, () -> follower, 6, 138);
        this.resetRuntime();
        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        intake.close(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, PreloadShoot),
                                shooter.flywheel(true),
                                shooter.turretOff(false),
                                new SequentialCommandGroup(
                                        new WaitCommand(1350),
                                        intake.open(),
                                        intake.collect()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup2, false),
                        new FollowPathCommand(follower, OpenGateFor2, false, 0.5).withTimeout(1500),
                        shooter.turretOff(false),


                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot2, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1200),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1100),
                        shooter.turretOff(false),
                        new WaitCommand(150),
                        new FollowPathCommand(follower, ShootGate1, true, 0.5).withTimeout(1000),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ShootGate2, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                ).withTimeout(1600)
                        ),
                        new WaitCommand(550),

                        intake.close(),

                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1100),
                        shooter.turretOff(false),
                        new WaitCommand(2050),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ShootGate2, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
                        intake.close(),


                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1100),
                        shooter.turretOff(false),
                        new WaitCommand(2050),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ShootGate2, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
                        intake.close(),



                        new FollowPathCommand(follower, Goto1, false),
                        shooter.turretOff(false),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot1, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(750),
                                        new WaitCommand(200),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),

//                        new FollowPathCommand(follower, Pickup3, true),
//                        shooter.turretOff(false),
//
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(follower, Shoot3, true),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1650),
//                                        intake.open()
//                                )
//                        ),
//                        new WaitCommand(550),
////                        shooter.turretOff(true),
//                        intake.close(),



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
        telemetryData.addData("Auto Time", this.getRuntime());
        telemetryData.update();

        Memory.robotHeading = follower.getHeading();
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotPose = follower.getPose();

//        Log.d("Drive power")
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
        telemetryData.addData("Auto Time", this.getRuntime());

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}
