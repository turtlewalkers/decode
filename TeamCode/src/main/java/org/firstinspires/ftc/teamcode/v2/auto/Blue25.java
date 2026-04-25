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
public class Blue25 extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;
    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    public static int T = 1;
    private final Pose Start = new Pose(28.4, 129.4, Math.toRadians(134));
    private final Pose PreloadScore = new Pose(54, 84, Math.toRadians(145));
    private final Pose TurnPreloadScore = new Pose( 54, 84, Math.toRadians(200));
    private final Pose Collect2Control = new Pose(55, 55, Math.toRadians(180));
    private final Pose Collect2 = new Pose(20, 59, Math.toRadians(180));
    private final Pose Score2 = new Pose(58, 82, Math.toRadians(220));
    private final Pose CollectGateControl = new Pose(45.5, 68.5, Math.toRadians(148.5));

    private final Pose CollectGateTurn = new Pose(27.5, 61, Math.toRadians(165));
    private final Pose CollectGate = new Pose(14.4, 61.4, Math.toRadians(153));
    private final Pose GateShoot = new Pose(60, 80, Math.toRadians(205));
    private final Pose GateShootControl = new Pose(31, 55, Math.toRadians(200));

    private final Pose GateShoot1 = new Pose(58, 82, Math.toRadians(170));
    private final Pose GateShootLast = new Pose(58, 103, Math.toRadians(230));

    private final Pose Collect1Control = new Pose(30.5, 80, Math.toRadians(180));
    private final Pose Collect1 = new Pose(22, 82.5, Math.toRadians(180));
    private final Pose Score1 = new Pose(58, 82, Math.toRadians(190));

    private final Pose Collect3Control = new Pose(40.5, 29, Math.toRadians(180));
    private final Pose Collect3 = new Pose(24, 35, Math.toRadians(180));
    private final Pose Score3 = new Pose(62, 100, Math.toRadians(250));




    private Path PreloadShoot;

    private Path TurnPS;
    private Path Intake2;
    private Path Shoot2;
    private Path GateIntakeFirst;

    private Path TurnGS;

    private Path GateIntakeP1;
    private Path GateIntakeP2;
    private PathChain GateIntakeP12;
    private Path GateScore;

    private Path GateScore1;

    private Path GateIntake1;
    private Path GateScoreLast;
    private Path Intake1;
    private Path Shoot1;


    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        // run 1
        PreloadShoot = new Path(new BezierLine(Start, PreloadScore));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), PreloadScore.getHeading());
        PreloadShoot.setTimeoutConstraint(50);

        TurnPS = new Path(new BezierLine(PreloadScore, TurnPreloadScore));
        TurnPS.setLinearHeadingInterpolation(PreloadScore.getHeading(), TurnPreloadScore.getHeading());
        TurnPS.setTimeoutConstraint(50);

        Intake2 = new Path(new BezierCurve(TurnPreloadScore, Collect2Control, Collect2));
        Intake2.setLinearHeadingInterpolation(TurnPreloadScore.getHeading(), Collect2.getHeading());
        Intake2.setTimeoutConstraint(50);

        Shoot2 = new Path(new BezierLine(Collect2, Score2));
        Shoot2.setLinearHeadingInterpolation(Collect2.getHeading(), Score2.getHeading());
        Shoot2.setTimeoutConstraint(50);


        GateIntakeFirst = new Path(new BezierCurve(Score2, CollectGateControl, CollectGate));
        GateIntakeFirst.setLinearHeadingInterpolation(Score2.getHeading(), CollectGate.getHeading());
        GateIntakeFirst.setTimeoutConstraint(50);

        GateScore1 = new Path(new BezierLine(CollectGate, GateShoot1));
        GateScore1.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShoot.getHeading());
        GateScore1.setTimeoutConstraint(50);

        TurnGS = new Path(new BezierLine(GateShoot1, GateShoot1));
        TurnGS.setLinearHeadingInterpolation(GateShoot.getHeading(), GateShoot1.getHeading());
        TurnGS.setTimeoutConstraint(50);


        Intake1 = new Path(new BezierCurve(GateShoot1, Collect1Control, Collect1));
        Intake1.setLinearHeadingInterpolation(GateShoot1.getHeading(), Collect1.getHeading());
        Intake1.setTimeoutConstraint(50);

        Shoot1 = new Path(new BezierLine(Collect1, Score1));
        Shoot1.setLinearHeadingInterpolation(Collect1.getHeading(), Score1.getHeading());
        Shoot1.setTimeoutConstraint(50);


        GateIntake1 = new Path(new BezierCurve(Score1, CollectGateControl, CollectGate));
        GateIntake1.setLinearHeadingInterpolation(Score1.getHeading(), CollectGate.getHeading());
        GateIntake1.setTimeoutConstraint(50);

        GateScore = new Path(new BezierCurve(CollectGate, GateShootControl, GateShoot));
        GateScore.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShoot.getHeading());
        GateScore.setTimeoutConstraint(50);

        GateIntakeP1 = new Path(new BezierCurve(GateShoot, CollectGateControl, CollectGateTurn));
        GateIntakeP1.setLinearHeadingInterpolation(GateShoot.getHeading(), CollectGateTurn.getHeading());
        GateIntakeP1.setTimeoutConstraint(50);

        GateIntakeP2 = new Path(new BezierLine(CollectGateTurn, CollectGate));
        GateIntakeP2.setLinearHeadingInterpolation(CollectGateTurn.getHeading(), CollectGate.getHeading());
        GateIntakeP2.setTimeoutConstraint(50);

        GateIntakeP12 = new PathChain(
                GateIntakeP1,
                GateIntakeP2
        );

//        GateScore = new Path(new BezierLine(CollectGate, GateShoot));
//        GateScore.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShoot.getHeading());
//        GateScore.setTimeoutConstraint(50);

        GateScoreLast = new Path(new BezierLine(CollectGate, GateShootLast));
        GateScoreLast.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShootLast.getHeading());
        GateScoreLast.setTimeoutConstraint(50);
//
//        Intake3 = new Path(new BezierCurve(Score1, Collect3Control, Collect3));
//        Intake3.setLinearHeadingInterpolation(Score1.getHeading(), Collect3.getHeading());
//        Intake3.setTimeoutConstraint(50);
//
//        Shoot3 = new Path(new BezierLine(Collect3, Score3));
//        Shoot3.setLinearHeadingInterpolation(Collect3.getHeading(), Score3.getHeading());
//        Shoot3.setTimeoutConstraint(50);
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
                new SequentialCommandGroup (
                        shooter.SWMon(),
                        intake.shootStop(),
                        shooter.turretOff(false),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, PreloadShoot, true).withTimeout(1100),
                                new SequentialCommandGroup(
                                        intake.collectStart(),
                                        new WaitCommand(150),
                                        intake.shootStart()

                                )
                        ),
                        intake.shootStop(),
                        intake.collectStart(),
                        shooter.SWMoff(),
                        new FollowPathCommand(follower, TurnPS, true).withTimeout(400),
                        new FollowPathCommand(follower, Intake2, true).withTimeout(1100),
                        new FollowPathCommand(follower, Shoot2, true).withTimeout(1100),
                        new WaitCommand(50),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),


                        new FollowPathCommand(follower, GateIntakeFirst, true).withTimeout(1100),
                        new WaitCommand(1300),
                        intake.collectStop(),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateScore1, true).withTimeout(1100),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        intake.collectStop()
                                )
                        ),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, TurnGS, true).withTimeout(400),
                        new FollowPathCommand(follower, Intake1, true).withTimeout(1100),
                        new FollowPathCommand(follower, Shoot1, true).withTimeout(1100),
                        new WaitCommand(50),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),


                        new FollowPathCommand(follower, GateIntake1, true).withTimeout(1100),
                        new WaitCommand(1200),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        intake.collectStop()
                                )
                        ),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),


                        new FollowPathCommand(follower, GateIntakeP12, false).withTimeout(1700),
                        new WaitCommand(2100),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        intake.collectStop()
                                )
                        ),
                        new WaitCommand(250),
                        intake.shootStart(),
                        new WaitCommand(350),
                        intake.shootStop(),
                        intake.collectStart(),


//                        new FollowPathCommand(follower, GateIntakeP12, true).withTimeout(1700),
//                        new WaitCommand(1800),
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(follower, GateScore, true).withTimeout(1100),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1000),
//                                        intake.collectStop()
//                                )
//                        ),
//                        new WaitCommand(100),
//                        intake.shootStart(),
//                        new WaitCommand(350),
//                        intake.shootStop(),
//                        intake.collectStart()

                        new FollowPathCommand(follower, GateIntakeP12, true).withTimeout(1700),
                        new WaitCommand(2100),
                        new FollowPathCommand(follower, GateScoreLast, true).withTimeout(1100),
                        new WaitCommand(250),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart()
                        //new FollowPathCommand(follower, Intake3, true).withTimeout(2000),
                        //new FollowPathCommand(follower, Shoot3, true).withTimeout(1100)
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