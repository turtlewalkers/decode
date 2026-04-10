package org.firstinspires.ftc.teamcode.v2.auto;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
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
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.ShooterMove;

@Autonomous
public class Blue24 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    public static int T = 1;
    private final Pose Start = new Pose(28.4, 129.4, Math.toRadians(134));
    private final Pose PreloadScore = new Pose(60, 80, Math.toRadians(220));
    private final Pose Collect2Control = new Pose(56, 60, Math.toRadians(180));
    private final Pose Collect2 = new Pose(18, 65, Math.toRadians(180));
    private final Pose Score2 = new Pose(60, 80, Math.toRadians(220));
    private final Pose CollectGateControl = new Pose(45, 55, Math.toRadians(148.5));
    private final Pose CollectGate = new Pose(14, 63, Math.toRadians(146));
    private final Pose GateShoot = new Pose(60, 80, Math.toRadians(220));

    private final Pose GateShootLast = new Pose(60, 80, Math.toRadians(150));
    private final Pose Collect3 = new Pose(18, 35, Math.toRadians(180));
    private final Pose Collect1Control = new Pose(30.5, 85, Math.toRadians(180));
    private final Pose Collect1 = new Pose(18, 82.5, Math.toRadians(180));



    private Path PreloadShoot;
    private Path Intake2;
    private Path Shoot2;
    private Path GateIntake;
    private Path GateScore;
    private Path GateScoreLast;
    private Path Intake1;
    private PathChain tatawireless2;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        PreloadShoot = new Path(new BezierLine(Start, PreloadScore));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), PreloadScore.getHeading());
        PreloadShoot.setTimeoutConstraint(50);

        Intake2 = new Path(new BezierCurve(PreloadScore, Collect2Control, Collect2));
        Intake2.setLinearHeadingInterpolation(PreloadScore.getHeading(), Collect2.getHeading());
        Intake2.setTimeoutConstraint(50);

        Shoot2 = new Path(new BezierLine(Collect2, Score2));
        Shoot2.setLinearHeadingInterpolation(Collect2.getHeading(), Score2.getHeading());
        Shoot2.setTimeoutConstraint(50);

        GateIntake = new Path(new BezierCurve(Score2, CollectGateControl, CollectGate));
        GateIntake.setLinearHeadingInterpolation(Score2.getHeading(), CollectGate.getHeading());
        GateIntake.setTimeoutConstraint(50);

        GateScore = new Path(new BezierLine(CollectGate, GateShoot));
        GateScore.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShoot.getHeading());
        GateScore.setTimeoutConstraint(50);

        GateScoreLast = new Path(new BezierLine(CollectGate, GateShootLast));
        GateScoreLast.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShootLast.getHeading());
        GateScoreLast.setTimeoutConstraint(50);

        Intake1 = new Path(new BezierCurve(GateShoot, Collect1Control, Collect1));
        Intake1.setLinearHeadingInterpolation(GateShoot.getHeading(), Collect1.getHeading());
        Intake1.setTimeoutConstraint(50);
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
        shooter = new ShooterMove(hardwareMap, () -> follower, 6, 138);
        intake = new Intake(hardwareMap, () -> follower, 6, 138);
        this.resetRuntime();
        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, PreloadShoot, true).withTimeout(1100),
                        new WaitCommand(550),
                        new FollowPathCommand(follower, Intake2, true).withTimeout(1100),
                        new FollowPathCommand(follower, Shoot2, true).withTimeout(1100),
                        new WaitCommand(550),
                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, GateScoreLast, true).withTimeout(1100),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, Intake1, true).withTimeout(1100)
                        )
        );
    }


    @Override
    public void run() {
        super.run();
        if (T == 1) {
            this.resetRuntime();
            Memory.autoRan = true;
            Log.d("Reset Time", String.valueOf(Memory.autoRan));
            T++;
        }
        Memory.autoRan = true;


        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("Auto Time", this.getRuntime());
        Log.d("pose", String.valueOf(Memory.robotPose));
        if (Math.abs(follower.getHeading()) > 0.05) {
            Memory.robotHeading = follower.getHeading();
        }
        if (Math.abs(follower.getPose().getX()) > 0.05) {
            Memory.robotAutoX = follower.getPose().getX();
        }
        if (Math.abs(follower.getPose().getY()) > 0.05) {
            Memory.robotAutoY = follower.getPose().getY();
        }
        if (Math.abs(follower.getPose().getY()) > 0.05 && Math.abs(follower.getPose().getX()) > 0.05) {
            Memory.robotPose = follower.getPose();
        }
        telemetry.update();

//        Log.d("Drive power")
    }

    @Override
    public void end() {

        Log.d("Ypos", String.valueOf(Memory.robotAutoX));
        Log.d("Xpos", String.valueOf(Memory.robotAutoY));
        Log.d("Head", String.valueOf(Memory.robotHeading));
        Log.d("EPose", String.valueOf(Memory.robotPose));
        if (Math.abs(follower.getHeading()) > 0.05) {
            Memory.robotHeading = follower.getHeading();
        }
        if (Math.abs(follower.getPose().getX()) > 0.05) {
            Memory.robotAutoX = follower.getPose().getX();
        }
        if (Math.abs(follower.getPose().getY()) > 0.05) {
            Memory.robotAutoY = follower.getPose().getY();
        }
        if (Math.abs(follower.getPose().getY()) > 0.05 && Math.abs(follower.getPose().getX()) > 0.05) {
            Memory.robotPose = follower.getPose();
        }


        Memory.autoRan = true;
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("Auto Time", this.getRuntime());

        telemetry.update();
    }
}