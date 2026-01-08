package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

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
public class BlueGateTest extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private double StartTime;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(144-117, 128, Math.toRadians(135));

    private final Pose Paneer2 = new Pose(144-115+2-1, 121.5, Math.toRadians(135));
    private final Pose ScorePositiona = new Pose(144-85, 84, Math.toRadians(160));
    private final Pose ScorePosition = new Pose(144-88, 86, Math.toRadians(240));
    private final Pose Collect1 = new Pose(144-118, 84, Math.toRadians(180));
    private final Pose CollectGate = new Pose(15, 62, Math.toRadians(180-28));
    private final Pose LeaveGate = new Pose(8, 54, Math.toRadians(180-30));
    private final Pose Collect2 = new Pose(144-118, 60, Math.toRadians(180));
    private final Pose Collect3 = new Pose(144-118, 36, Math.toRadians(180));
    private final Pose Grab4Setup = new Pose(144-(126+2-1), 48-4-3, Math.toRadians(240));
    private final Pose Grab4 = new Pose(144-(130+2-1), 25-4-3, Math.toRadians(260));
    private final Pose GotoS4 = new Pose(144-(120-1), 28-3, Math.toRadians(260));
    private final Pose Collect4 = new Pose(6, 9, Math.toRadians(270));
    private final Pose byebye = new Pose(52, 76, Math.toRadians(225));
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, ToGate, GotoIntakeGate, GateIntake, ShootGate1, ShootGate2, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        Paneer = new Path(new BezierLine(Start, Paneer2));
        Paneer.setLinearHeadingInterpolation(Start.getHeading(), Paneer2.getHeading());
        Paneer.setTimeoutConstraint(50);

        PreloadShoot = new Path(new BezierLine(Paneer2, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Paneer2.getHeading(), ScorePosition.getHeading());
        PreloadShoot.setTimeoutConstraint(50);

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePositiona, Collect1))
                .setLinearHeadingInterpolation(ScorePositiona.getHeading(), Collect1.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GotoIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(44, 64),
                        CollectGate)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), CollectGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        ShootGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        CollectGate,
                        new Pose(14, 63),
                        LeaveGate)
                )
                .setLinearHeadingInterpolation(CollectGate.getHeading(), LeaveGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        ShootGate2 = follower.pathBuilder()
                .addPath(new BezierLine(LeaveGate, ScorePosition))
                .setLinearHeadingInterpolation(LeaveGate.getHeading(), ScorePosition.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(54, 57.5),
                        Collect2)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect2.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(54, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePosition))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePosition.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4Part1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab4))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4Part2 = follower.pathBuilder()
                .addPath(new BezierLine(Grab4Setup, Grab4))
                .setLinearHeadingInterpolation(Grab4Setup.getHeading(), Grab4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        ScorePosition,
                        new Pose(12, 70),
                        Collect4)
                )
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Collect4.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot4P1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect4, ScorePosition))
                .setLinearHeadingInterpolation(Collect4.getHeading(), ScorePosition.getHeading())
                .setTimeoutConstraint(50)
                .build();

        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .setTimeoutConstraint(50)
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = false;
        Memory.autoRan = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new Shooter(hardwareMap, () -> follower, 6, 138, true);
        intake = new Intake(hardwareMap, () -> follower, 6, 138);

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

                        new FollowPathCommand(follower, GotoIntakeGate, true).withTimeout(1100),
                        shooter.turretOff(false),
                        new WaitCommand(1000),

                        new WaitCommand(600),
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

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}