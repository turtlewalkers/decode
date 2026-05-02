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
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.ShooterMove;

import java.util.function.BooleanSupplier;

@Configurable
@Autonomous
public class Blue21Far extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private ShooterMove shooter;
    private double StartTime;

    private GoBildaPinpointDriver pinpoint;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    public static int T = 1;
    private final Pose Start = new Pose(55.3 , 8.52, Math.toRadians(90));
    private final Pose Collect3 = new Pose(14, 35, Math.toRadians(180));
    private final Pose Collect3Control = new Pose(40, 33, Math.toRadians(0));

    private final Pose Shoot = new Pose(55 , 13, Math.toRadians(180));
    private final Pose FarCollect = new Pose(12, 11, Math.toRadians(180));

    private final Pose FarCollect1 = new Pose(12, 11, Math.toRadians(175));

    private final Pose FarCollect2 = new Pose(12, 11, Math.toRadians(185));

    private final Pose DiagonalCollect = new Pose(14, 20, Math.toRadians(150));

    private final Pose Leave = new Pose(34, 11, Math.toRadians(180));




    private Path Intake3, Shoot3, FarIntake, FarShoot, DiagonalIntake, DiagonalShoot, LeaveZone, FarTurn1, FarTurn2;


    private boolean has3 = true;
    private BooleanSupplier supplier;
    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        follower.setMaxPower(1);

        // run 1
        Intake3 = new Path(new BezierCurve(Start, Collect3Control, Collect3));
        Intake3.setLinearHeadingInterpolation(Start.getHeading(), Collect3.getHeading());
        Intake3.setTimeoutConstraint(50);

        Shoot3 = new Path(new BezierLine(Collect3, Shoot));
        Shoot3.setLinearHeadingInterpolation(Collect3.getHeading(), Shoot.getHeading());
        Shoot3.setTimeoutConstraint(50);


        FarIntake = new Path(new BezierLine(Shoot, FarCollect));
        FarIntake.setLinearHeadingInterpolation(Shoot.getHeading(), FarCollect.getHeading());
        FarIntake.setTimeoutConstraint(50);

        FarShoot = new Path(new BezierLine(FarCollect2, Shoot));
        FarShoot.setLinearHeadingInterpolation(FarCollect2.getHeading(), Shoot.getHeading());
        FarShoot.setTimeoutConstraint(50);

        FarTurn1= new Path(new BezierLine(FarCollect, FarCollect1));
        FarTurn1.setLinearHeadingInterpolation(FarCollect.getHeading(), FarCollect1.getHeading());
        FarTurn1.setTimeoutConstraint(50);

        FarTurn2= new Path(new BezierLine(FarCollect1, FarCollect2));
        FarTurn2.setLinearHeadingInterpolation(FarCollect1.getHeading(), FarCollect2.getHeading());
        FarTurn2.setTimeoutConstraint(50);

        DiagonalIntake = new Path(new BezierLine(Shoot, DiagonalCollect));
        DiagonalIntake.setLinearHeadingInterpolation(Shoot.getHeading(), DiagonalCollect.getHeading());
        DiagonalIntake.setTimeoutConstraint(50);

        DiagonalShoot = new Path(new BezierLine(DiagonalCollect, Shoot));
        DiagonalShoot.setLinearHeadingInterpolation(DiagonalCollect.getHeading(), Shoot.getHeading());
        DiagonalShoot.setTimeoutConstraint(50);

        LeaveZone = new Path(new BezierLine(Shoot, Leave));
        LeaveZone.setLinearHeadingInterpolation(Shoot.getHeading(), Leave.getHeading());
        LeaveZone.setTimeoutConstraint(50);
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
        register(intake);
//        shooter.flywheelOff();
//        shooter.startTurret();
        buildpaths();

        double offset = 0;
        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup (
                        shooter.SWMoff(),
                        intake.shootStop(),
                        shooter.flywheel(true),
                        shooter.turretOff(false),
                        shooter.aimAt(Start, Start.getHeading()+Math.toRadians(2)),
                        new WaitCommand(1600),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, Intake3, true).withTimeout(1200),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, Shoot3, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),


                        new FollowPathCommand(follower, FarIntake, true).withTimeout(1200),
                        new FollowPathCommand(follower, FarTurn1, true).withTimeout(200),
                        new FollowPathCommand(follower, FarTurn2, true).withTimeout(200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, FarShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, DiagonalIntake, true).withTimeout(1200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, DiagonalShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, FarIntake, true).withTimeout(1200),
                        new FollowPathCommand(follower, FarTurn1, true).withTimeout(200),
                        new FollowPathCommand(follower, FarTurn2, true).withTimeout(200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, FarShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, DiagonalIntake, true).withTimeout(1200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, DiagonalShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, FarIntake, true).withTimeout(1200),
                        new FollowPathCommand(follower, FarTurn1, true).withTimeout(200),
                        new FollowPathCommand(follower, FarTurn2, true).withTimeout(200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, FarShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, DiagonalIntake, true).withTimeout(1200),
                        new WaitCommand(250),
                        shooter.aimAt(Shoot, Shoot.getHeading()+Math.toRadians(2)),
                        new FollowPathCommand(follower, DiagonalShoot, true).withTimeout(1200),
                        new WaitCommand(300),
                        intake.shootStart(),
                        new WaitCommand(500),
                        intake.shootStop(),
                        intake.collectStart(),

                        new FollowPathCommand(follower, LeaveZone, true).withTimeout(1200)
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