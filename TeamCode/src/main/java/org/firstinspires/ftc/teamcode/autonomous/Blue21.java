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
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

@Autonomous
public class Blue21 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(25, 126, Math.toRadians(135));
    private final Pose PreloadScore = new Pose(58, 74, Math.toRadians(200));
    private final Pose Stack2Score = new Pose(58.5, 73, Math.toRadians(190));
    private final Pose OpenGate = new Pose(12.75, 60, Math.toRadians(152));
    private final Pose CollectGate = new Pose(11, 57, Math.toRadians(150));
    private final Pose Gate1Score = new Pose(60.5, 72, Math.toRadians(205));
    private final Pose Stack1Score = new Pose(51, 79, Math.toRadians(245));
    private final Pose Stack3Score = new Pose(60, 97, Math.toRadians(245));
    private final Pose Collect1 = new Pose(144-125, 83, Math.toRadians(180));
    private final Pose Collect2 = new Pose(144-129, 59, Math.toRadians(180));
    private final Pose Collect3 = new Pose(144-128, 36, Math.toRadians(180));
    private final Pose LeaveZone = new Pose(51, 72, Math.toRadians(245));

    private double timer = 0;
    private Path PreloadShoot;
    private Path Paneer;
    private PathChain Goto1, Pickup1, Shoot1, IntakeGate1, FirstGateShoot, GateShoot, IntakeGate2, FirstGateIntake, GateIntake, ShootGate1, ShootGate2, ShootGate3, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;


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
                        new Pose(54, 58.5),
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
                        new Pose(36.5, 56.5),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Stack2Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        FirstGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(14.8, 58),
                        CollectGate)
                )
                .setLinearHeadingInterpolation(OpenGate.getHeading(), CollectGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        FirstGateShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        CollectGate,
                        new Pose(26.5, 50),
                        Gate1Score)
                )
                .setLinearHeadingInterpolation(CollectGate.getHeading(), Gate1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Gate1Score,
                        new Pose(36.5, 56.5),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Gate1Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(26, 56),
                        Gate1Score)
                )
                .setLinearHeadingInterpolation(OpenGate.getHeading(), Gate1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();


        Goto1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Gate1Score,
                        new Pose(45, 84),
                        Collect1)
                )
                .setLinearHeadingInterpolation((Gate1Score.getHeading()-45), Collect1.getHeading())
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
                        new Pose(57, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(Stack1Score.getHeading(), Collect3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, Stack3Score))
                .setLinearHeadingInterpolation(Collect3.getHeading(), Stack3Score.getHeading())
                .setTimeoutConstraint(50)
                .build();


        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(Stack3Score, LeaveZone))
                .setLinearHeadingInterpolation(Stack3Score.getHeading(), LeaveZone.getHeading())
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
                        new WaitCommand(600),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup2, false),
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

                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1100),
                        shooter.turretOff(false),
                        new WaitCommand(150),
                        new FollowPathCommand(follower, FirstGateIntake, true, 0.5),
                        new WaitCommand(500),
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
                        shooter.turretOff(false),
                        new WaitCommand(1950),

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
                        shooter.turretOff(false),
                        new WaitCommand(1950),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1600),
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
                                        new WaitCommand(350),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup3, true),
                        shooter.turretOff(false),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot3, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1750),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(550),
//                        shooter.turretOff(true),
                        intake.close(),



                        shooter.turretOff(true),
//                        new FollowPathCommand(follower, tatawireless, true),
                        new WaitCommand(100),
                        new InstantCommand(() -> shooter.flywheel(false))
                )
        );
    }


    @Override

    public void run() {
        if (timer == 0) {
            this.resetRuntime();
            timer = 1;
        }
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
