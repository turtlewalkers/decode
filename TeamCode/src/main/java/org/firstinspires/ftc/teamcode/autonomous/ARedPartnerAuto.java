package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
public class ARedPartnerAuto extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(144-26.7, 128.2, Math.toRadians(180-135));
    private final Pose PreloadScore = new Pose(144-61, 72.4, Math.toRadians(540-200));
    private final Pose Stack2Score = new Pose(144-57, 78, Math.toRadians(540-190));
    private final Pose OpenGate = new Pose(144-14, 64, Math.toRadians(180-150));
    private final Pose CollectGate = new Pose(144-13, 56, Math.toRadians(180-125));
    private final Pose Gate1Score = new Pose(144-57, 78, Math.toRadians(540-190));
    private final Pose Turny1 = new Pose(144-59, 78, Math.toRadians(180-165));
    private final Pose Stack1Score = new Pose(144-56, 82, Math.toRadians(540-245));
    private final Pose Stack3Score = new Pose(144-62.5, 102, Math.toRadians(540-235));
    private final Pose Collect1 = new Pose(123, 83, Math.toRadians(0));
    private final Pose Collect2 = new Pose(127, 60, Math.toRadians(0));
    private final Pose Collect3 = new Pose(127, 36, Math.toRadians(0));
    private final Pose Turny3 = new Pose(127, 36, Math.toRadians(540-220));
    private final Pose LeaveZone = new Pose(144-51, 72, Math.toRadians(540-245));
    private final Pose GateOpen = new Pose(144-16, 70, Math.toRadians(0));

    private double timer = 0;
    private PathChain ThirdGateSchoot, PreloadShoot, Goto1,OpenGateAfter1,  Pickup1, Shoot1, IntakeGate1, FirstGateShoot, GateShoot, IntakeGate2, FirstGateIntake, GateIntake, ShootGate1, ShootGate2, ShootGate3, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, Goto4Part1, Goto4Part2, Goto4, Shoot4P1, Shoot4P2, tatawireless, tatawireless2;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        PreloadShoot = follower.pathBuilder()
                .addPath(new BezierLine(Start, PreloadScore))
                .setLinearHeadingInterpolation(Start.getHeading(), PreloadScore.getHeading())
                .setTimeoutConstraint(50)
                .build();


        Pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        PreloadScore,
                        new Pose(144-54, 58.5),
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
                        new Pose(144-36, 59.5),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Stack2Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        FirstGateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(144-17, 60),
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
        ThirdGateSchoot = follower.pathBuilder()
                .addPath(new BezierLine(CollectGate, Stack3Score))
                .setLinearHeadingInterpolation(CollectGate.getHeading(), Stack3Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Gate1Score,
                        new Pose(144-40, 62.5),
                        OpenGate)
                )
                .setLinearHeadingInterpolation(Gate1Score.getHeading(), OpenGate.getHeading())
                .setTimeoutConstraint(50)
                .build();

        GateShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        OpenGate,
                        new Pose(144-18.3, 60.31578947368421),
                        new Pose(144-21.08947368421052, 62.013157894736835),
                        Gate1Score)
                )
                .setLinearHeadingInterpolation(OpenGate.getHeading(), Gate1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        OpenGateAfter1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Collect1,
                        new Pose(144-27, 75),
                        GateOpen)
                )
                .setLinearHeadingInterpolation((Collect1.getHeading()), GateOpen.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(GateOpen, Stack1Score))
                .setLinearHeadingInterpolation(GateOpen.getHeading(), Stack1Score.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Stack1Score,
                        new Pose(144-57, 31),
                        Collect3)
                )
                .setLinearHeadingInterpolation(Stack1Score.getHeading(), Collect3.getHeading())
                .setTimeoutConstraint(50)
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Collect3,
                        new Pose(144-44, 50),
                        Stack3Score)
                )
                .setLinearHeadingInterpolation(Collect3.getHeading(), Stack3Score.getHeading())
                .setTimeoutConstraint(50)
                .build();
        Goto1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Gate1Score,
                        new Pose(144-45, 84),
                        Collect1)
                )
                .setLinearHeadingInterpolation((Gate1Score.getHeading()+45), Collect1.getHeading())
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
        Memory.allianceRed = false;
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
                                new FollowPathCommand(follower, PreloadShoot, 1),
                                shooter.flywheel(true),
                                shooter.turretOff(false), //change to false when turret is fixed
                                new SequentialCommandGroup(
                                        new WaitCommand(1650),
                                        intake.open(),
                                        intake.collect()
                                )
                        ),
                        new WaitCommand(800),
//                        shooter.turretOff(true),
                        intake.close(),

                        new FollowPathCommand(follower, Pickup2, false),
                        shooter.turretOff(false), //change


                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot2, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1200),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(800),
//                        shooter.turretOff(true),

                        intake.close(),
                        new FollowPathCommand(follower, Goto1, false),
                        shooter.turretOff(false), //change
                        new FollowPathCommand(follower, OpenGateAfter1, false),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, Shoot1, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(750),
                                        new WaitCommand(350),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(800),
//                        shooter.turretOff(true),
                        intake.close(),


                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1100),
                        shooter.turretOff(false), //s
                        new WaitCommand(450),
                        new FollowPathCommand(follower, FirstGateIntake, true, 0.5).withTimeout(500),
                        new WaitCommand(950),
//                        intake.stop(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, FirstGateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.collect(),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(800),
                        intake.close(),
                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1100),
                        new WaitCommand(450),
                        shooter.turretOff(false), //b
                        new FollowPathCommand(follower, FirstGateIntake, true, 0.5).withTimeout(500),
                        new WaitCommand(950),
//                        intake.stop(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, FirstGateShoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.collect(),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(800),
                        intake.close(),
                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1100),
                        new WaitCommand(450),
                        shooter.turretOff(false), //b
                        new FollowPathCommand(follower, FirstGateIntake, true, 0.5).withTimeout(500),
                        new WaitCommand(950),
//                        intake.stop(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, ThirdGateSchoot, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(1500),
                                        intake.collect(),
                                        intake.open()
                                )
                        ),
                        new WaitCommand(800),
                        intake.close(),





                        shooter.turretOff(false),
                        new FollowPathCommand(follower, tatawireless, true),
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
//        Memory.autoRan = true;

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

