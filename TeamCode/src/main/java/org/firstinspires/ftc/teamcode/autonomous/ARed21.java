package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

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
public class ARed21 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    public static int T;
    private final Pose Start = new Pose(144-26.7, 128.2, Math.toRadians(180-135));
    private final Pose PreloadScore = new Pose(144-61, 72.4, Math.toRadians(540-200));
    private final Pose Stack2Score = new Pose(145.5-57, 78, Math.toRadians(540-190));
    private final Pose OpenGate = new Pose(144-14, 63, Math.toRadians(180-150));
    private final Pose CollectGate = new Pose(144-13, 56, Math.toRadians(180-125));
    private final Pose Gate1Score = new Pose(145.5-57, 78, Math.toRadians(540-190));
    private final Pose Turny1 = new Pose(145.5-59, 78, Math.toRadians(180-165));
    private final Pose Stack1Score = new Pose(145.5-56, 82, Math.toRadians(540-245));
    private final Pose Stack3Score = new Pose(145.5-62.5, 102, Math.toRadians(540-235));
    private final Pose Collect1 = new Pose(125, 84, Math.toRadians(0));
    private final Pose Collect2 = new Pose(127, 60, Math.toRadians(0));
    private final Pose Collect3 = new Pose(128, 36, Math.toRadians(0));
    private final Pose Turny3 = new Pose(127, 36, Math.toRadians(540-220));
    private final Pose LeaveZone = new Pose(145.5-51, 72, Math.toRadians(540-245));


    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Turn1, Turn3, Goto1, Pickup1, Shoot1, IntakeGate1, FirstGateShoot, GateShoot, IntakeGate2, FirstGateIntake, GateIntake, ShootGate1, ShootGate2, ShootGate3, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2                                                                                                                                                                                                                                                                                                ;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        PreloadShoot = new Path(new BezierLine(Start, PreloadScore));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), PreloadScore.getHeading());
        PreloadShoot.setTimeoutConstraint(50);



        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PreloadScore,
                        new Pose(145.5-54, 59.5),
                        Collect2)
                )
                .setLinearHeadingInterpolation(PreloadScore.getHeading(), Collect2.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, Stack2Score))
                .setLinearHeadingInterpolation(Collect2.getHeading(), Stack2Score.getHeading())
                .setTimeoutConstraint(50)
                .build();




        IntakeGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Stack2Score,
                        new Pose(145.5-36, 62.5),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Stack2Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        FirstGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(145.5-17, 58),
                        CollectGate)
                )
                .setLinearHeadingInterpolation(OpenGate.getHeading(), CollectGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        FirstGateShoot = follower.pathBuilder()
                .addPath(new BezierLine(CollectGate, Gate1Score))
                .setLinearHeadingInterpolation(CollectGate.getHeading(), Gate1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Gate1Score,
                        new Pose(145.5-40, 62),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Gate1Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(145.5-18.3, 60.31578947368421),
                        new Pose(145.5-21.08947368421052, 62.013157894736835),
                        Gate1Score)
                )
                .setLinearHeadingInterpolation(OpenGate.getHeading(), Gate1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();
        Turn1 = follower.pathBuilder()
                .addPath(new BezierLine(Gate1Score, Turny1))
                .setLinearHeadingInterpolation(Gate1Score.getHeading(), Turny1.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(Turny1, Collect1))
                .setLinearHeadingInterpolation(Turny1.getHeading(), Collect1.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, Stack1Score))
                .setLinearHeadingInterpolation(Collect1.getHeading(), Stack1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Stack1Score,
                        new Pose(145.5-57, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(Stack1Score.getHeading(), Collect3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Turn3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, Turny3))
                .setLinearHeadingInterpolation(Collect3.getHeading(), Turny3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Turny3, Stack3Score))
                .setLinearHeadingInterpolation(Turny3.getHeading(), Stack3Score.getHeading())
                .setTimeoutConstraint(50)
                .build();


        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(Stack3Score, LeaveZone))
                .setLinearHeadingInterpolation(Stack3Score.getHeading(), LeaveZone.getHeading())
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
        Memory.allianceRed = true;
        Memory.autoRan = true;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new ShooterMove(hardwareMap, () -> follower, 138, 138, true);
        intake = new Intake(hardwareMap, () -> follower, 138, 138);
        this.resetRuntime();
        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        intake.close(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, PreloadShoot),
                                shooter.flywheel(true),
                                shooter.turretOff(false), //change
                                new SequentialCommandGroup(
                                        new WaitCommand(1650),
                                        intake.open(),
                                        intake.collect()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup2, false),


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

                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1100),
                        new WaitCommand(150),
                        new FollowPathCommand(follower, FirstGateIntake, true, 0.5),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, FirstGateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),

                        intake.close(),

                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1750),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
                        intake.close(),

                        new FollowPathCommand(follower, GateIntake, true).withTimeout(1100),
                        new WaitCommand(1750),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
                        intake.close(),

                        new FollowPathCommand(follower, Goto1, false),
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

                        new FollowPathCommand(follower, Pickup3, true),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot3, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1850),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),



//                        new FollowPathCommand(follower, tatawireless, true),
                        new WaitCommand(100),
                        new InstantCommand(() -> shooter.flywheel(false))
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

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}
