package org.firstinspires.ftc.teamcode.v2.auto;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.ShooterMove;

import java.util.function.BooleanSupplier;

@Configurable
@Autonomous
public class Blue21Safe extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;

    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    public static int T = 1;
    private final Pose Start = new Pose(27.3, 127.5, Math.toRadians(134));
    private final Pose PreloadScore = new Pose( 57, 75, Math.toRadians(134));

    private final Pose TurnPreloadScore = new Pose( 57, 75, Math.toRadians(190));
    private final Pose Collect2Control = new Pose(38, 59, Math.toRadians(0));
    private final Pose Collect2 = new Pose(16, 59, Math.toRadians(180));
    private final Pose Score2 = new Pose(52, 80, Math.toRadians(220));

    private final Pose CollectGateControl2 = new Pose(30, 57, Math.toRadians(0));
    private final Pose CollectGate1Control = new Pose(28, 57, Math.toRadians(0));
    private final Pose CollectGateCycleControl = new Pose(28, 57, Math.toRadians(0));
    private final Pose CollectGateTurn = new Pose(24, 61, Math.toRadians(150));
    private final Pose CollectGate = new Pose(11.8, 59.5, Math.toRadians(150));


    private final Pose GateShootLeave = new Pose(13.5, 57.5, Math.toRadians(205));
    private final Pose GateShoot = new Pose(53, 80, Math.toRadians(205));
    private final Pose GateShootControl = new Pose(28, 55, Math.toRadians(0));

    private final Pose GateShoot1 = new Pose(52, 82, Math.toRadians(170));
    private final Pose GateShootLast = new Pose(54, 103, Math.toRadians(220));

    private final Pose Collect1Control = new Pose(30.5, 87, Math.toRadians(0));
    private final Pose Collect1 = new Pose(24, 84, Math.toRadians(180));
    private final Pose Score1 = new Pose(58, 82, Math.toRadians(190));

//    private final Pose Collect3Control = new Pose(40.5, 29, Math.toRadians(180));
//    private final Pose Collect3 = new Pose(24, 35, Math.toRadians(180));
//    private final Pose Score3 = new Pose(62, 100, Math.toRadians(250));




    private Path PreloadShoot;

    private Path GateScoreTurn;
    private Path Intake2;
    private Path Shoot2;
    private Path GateIntake2;

    private Path TurnGateShoot1;
    private Path GateIntakeCycle;

    private Path FinishGateIntake;
    private Path GateScore;

    private Path TurnPreloadShoot;
    private Path GateScore1;

    private Path GateIntake1;
    private Path GateScoreEnd1;
    private Path Intake1;
    private Path Shoot1;
    private PathChain IntakeGate1, IntakeGate2, IntakeGateCycle, GateScoreFull, GateScoreFull1, GateScoreEnd;

    private boolean has3 = true;
    private BooleanSupplier supplier;
    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        // run 1
        PreloadShoot = new Path(new BezierLine(Start, PreloadScore));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), PreloadScore.getHeading());
        PreloadShoot.setTimeoutConstraint(50);

        Intake2 = new Path(new BezierCurve(TurnPreloadScore, Collect2Control, Collect2));
        Intake2.setLinearHeadingInterpolation(TurnPreloadScore.getHeading(), Collect2.getHeading());
        Intake2.setTimeoutConstraint(50);

        Shoot2 = new Path(new BezierLine(Collect2, Score2));
        Shoot2.setLinearHeadingInterpolation(Collect2.getHeading(), Score2.getHeading());
        Shoot2.setTimeoutConstraint(50);

        FinishGateIntake = new Path(new BezierLine(CollectGateTurn, CollectGate));
        FinishGateIntake.setLinearHeadingInterpolation(CollectGateTurn.getHeading(), CollectGate.getHeading());
        FinishGateIntake.setTimeoutConstraint(50);

        GateIntake2 = new Path(new BezierCurve(Score2, CollectGateControl2, CollectGateTurn));
        GateIntake2.setLinearHeadingInterpolation(Score2.getHeading(), CollectGateTurn.getHeading());
        GateIntake2.setTimeoutConstraint(50);

        IntakeGate2 = new PathChain(GateIntake2, FinishGateIntake);

        GateIntake1 = new Path(new BezierCurve(Score1, CollectGate1Control, CollectGateTurn));
        GateIntake1.setLinearHeadingInterpolation(Score1.getHeading(), CollectGateTurn.getHeading());
        GateIntake1.setTimeoutConstraint(50);

        GateIntakeCycle = new Path(new BezierCurve(GateShoot, CollectGateCycleControl, CollectGateTurn));
        GateIntakeCycle.setLinearHeadingInterpolation(GateShoot.getHeading(), CollectGateTurn.getHeading());
        GateIntakeCycle.setTimeoutConstraint(50);

        IntakeGateCycle = new PathChain(GateIntakeCycle, FinishGateIntake);

        IntakeGate1 = new PathChain(GateIntake1, FinishGateIntake);

        GateScore1 = new Path(new BezierLine(GateShootLeave, GateShoot1));
        GateScore1.setLinearHeadingInterpolation(GateShootLeave.getHeading(), GateShoot.getHeading());
        GateScore1.setTimeoutConstraint(50);

        TurnGateShoot1 = new Path(new BezierLine(GateShoot1, GateShoot1));
        TurnGateShoot1.setLinearHeadingInterpolation(GateShoot.getHeading(), GateShoot1.getHeading());
        TurnGateShoot1.setTimeoutConstraint(50);


        Intake1 = new Path(new BezierCurve(GateShoot1, Collect1Control, Collect1));
        Intake1.setLinearHeadingInterpolation(GateShoot1.getHeading(), Collect1.getHeading());
        Intake1.setTimeoutConstraint(50);

        Shoot1 = new Path(new BezierLine(Collect1, Score1));
        Shoot1.setLinearHeadingInterpolation(Collect1.getHeading(), Score1.getHeading());
        Shoot1.setTimeoutConstraint(50);

        GateScoreTurn = new Path(new BezierLine(CollectGate, GateShootLeave));
        GateScoreTurn.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShootLeave.getHeading());
        GateScoreTurn.setTimeoutConstraint(50);

        GateScore = new Path(new BezierLine(GateShootLeave, GateShoot));
        GateScore.setLinearHeadingInterpolation(GateShootLeave.getHeading(), GateShoot.getHeading());
        GateScore.setTimeoutConstraint(50);
        GateScoreFull = new PathChain(GateScoreTurn, GateScore);
        GateScoreFull1 = new PathChain(GateScoreTurn, GateScore1);

//        GateScore = new Path(new BezierLine(CollectGate, GateShoot));
//        GateScore.setLinearHeadingInterpolation(CollectGate.getHeading(), GateShoot.getHeading());
//        GateScore.setTimeoutConstraint(50);

        GateScoreEnd1 = new Path(new BezierLine(GateShootLeave, GateShootLast));
        GateScoreEnd1.setLinearHeadingInterpolation(GateShootLeave.getHeading(), GateShootLast.getHeading());
        GateScoreEnd1.setTimeoutConstraint(50);
        GateScoreEnd = new PathChain(GateScoreTurn, GateScoreEnd1);
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
        register(intake);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        shooter = new ShooterMove(hardwareMap, () -> follower, 6, 138);
        intake = new Intake(hardwareMap, () -> follower, 6, 138);
        this.resetRuntime();
//        shooter.flywheelOff();
//        shooter.startTurret();
        buildpaths();

        double offset = -4.5;
        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup (
                        shooter.aimAt(PreloadScore, (PreloadScore.getHeading()+Math.toRadians(offset))),
                        shooter.SWMoff(),
                        intake.shootStop(),
                        shooter.flywheel(true),
                        shooter.turretOff(false),
                        shooter.aimAt(PreloadScore, PreloadScore.getHeading()+Math.toRadians(offset)),
                        new FollowPathCommand(follower, PreloadShoot, true).withTimeout(1100),
                        shooter.aimAt(PreloadScore, PreloadScore.getHeading()+Math.toRadians(offset)),
                        new WaitCommand(100),
                        intake.collectStart(),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, Intake2, true).withTimeout(1400),
                        shooter.aimAt(Score2, Score2.getHeading()+Math.toRadians(offset) ),
                        new FollowPathCommand(follower, Shoot2, true).setGlobalMaxPower(1).withTimeout(1100),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),


                        new FollowPathCommand(follower, IntakeGate2, true).withTimeout(1200),
                        new ParallelRaceGroup(
                                new WaitCommand(1400),
                                new WaitUntilCommand(() -> intake.getBallCount() >= 3)
                        ),
                        new WaitCommand(100),
                        intake.collectStop(),
                        new ParallelCommandGroup(
                                shooter.aimAt(GateShoot1, GateShoot.getHeading()+Math.toRadians(offset)),
                                new FollowPathCommand(follower, GateScoreFull1, true).withTimeout(1100),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        intake.collectStop()
                                )
                        ),
//                        shooter.clearFixedAngle(),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),
                        shooter.clearFixedAngle(),


                        new FollowPathCommand(follower, TurnGateShoot1, true).withTimeout(400),
                        new FollowPathCommand(follower, Intake1, true).withTimeout(1100),
                        shooter.aimAt(Score1, Score1.getHeading()+Math.toRadians(offset)),
                        new FollowPathCommand(follower, Shoot1, true).withTimeout(1100),
//                        shooter.clearFixedAngle(),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),
                        shooter.clearFixedAngle(),


                        new FollowPathCommand(follower, IntakeGate1, true).withTimeout(1200),
                        new ParallelRaceGroup(
                                new WaitCommand(1700),
                                new WaitUntilCommand(() -> intake.getBallCount() == 3)
                        ),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                shooter.aimAt(GateShoot, GateShoot.getHeading()+Math.toRadians(offset)),
                                new FollowPathCommand(follower, GateScoreFull, true).withTimeout(1700),
                                new SequentialCommandGroup(
                                        new WaitCommand(900),
                                        intake.collectStop()
                                )
                        ),
//                        shooter.clearFixedAngle(),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),
                        shooter.clearFixedAngle(),



                        new FollowPathCommand(follower, IntakeGateCycle, true).withTimeout(1200),
                        new ParallelRaceGroup(
                                new WaitCommand(1800),
                                new WaitUntilCommand(() -> intake.getBallCount() >= 3)
                        ),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                shooter.aimAt(GateShoot, GateShoot.getHeading()+Math.toRadians(offset)),
                                new FollowPathCommand(follower, GateScoreFull, true).withTimeout(1500),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        intake.collectStop()
                                )
                        ),
//                        shooter.clearFixedAngle(),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart(),
//                        shooter.clearFixedAngle(),


//                        new FollowPathCommand(follower, IntakeGateCycle, false).withTimeout(1700),
//                        new WaitCommand(1700),
//                        new ParallelCommandGroup(
//                                shooter.aimAt(GateShoot, GateShoot.getHeading()),
//                                new FollowPathCommand(follower, GateScoreFull, true).withTimeout(1500),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(1000),
//                                        intake.collectStop()
//                                )
//                        ),
//                        new WaitCommand(50),
//                        intake.shootStart(),
//                        new WaitCommand(350),
//                        intake.shootStop(),
//                        intake.collectStart(),
//                        shooter.clearFixedAngle(),

                        shooter.aimAt(GateShootLast, GateShootLast.getHeading()+Math.toRadians(offset)),
                        new FollowPathCommand(follower, IntakeGateCycle, true).withTimeout(1200),
                        new ParallelRaceGroup(
                                new WaitCommand(1600),
                                new WaitUntilCommand(() -> intake.getBallCount() == 3)
                        ),
                        new WaitCommand(100),
                        //new WaitCommand()
                        //int temp = intake.getBallCount() == 3 ? elapsedTime :
                        new FollowPathCommand(follower, GateScoreEnd, true).withTimeout(1800),
//                        shooter.clearFixedAngle(),
                        new WaitCommand(150),
                        intake.shootStart(),
                        new WaitCommand(400),
                        intake.shootStop(),
                        intake.collectStart()
//                        shooter.clearFixedAngle()

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
//        has3 = intake.getBallCount() == 3;
//        supplier = () -> intake.getBallCount() == 3;
//        Log.d("Supplier", String.valueOf(supplier.getAsBoolean()));

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