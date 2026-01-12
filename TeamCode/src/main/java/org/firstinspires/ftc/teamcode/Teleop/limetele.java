package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;
import org.firstinspires.ftc.teamcode.subsystems.LimeMoving;

import java.util.List;

@Config
@TeleOp
public class limetele extends CommandOpMode {

    Follower follower;
    TelemetryData telemetryData;
    private GamepadEx gamepad, gamepadOffset;
    private Intake intake;
    private ShooterMove shooter;
    private LimeMoving limeMoving;
    public static double shooterX, shooterY, gateX, gateY;
    private double multiplier = 1;
    private Path Park, Stay;
    private Pose end, start, relocalize;
    List<LynxModule> allHubs;
    private DcMotorEx turret;
    private PIDController turretPID;
    public static double TICKS_PER_DEG = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
    public static double TURRET_MIN = -90;
    public static double TURRET_MAX = 240;
    public static double kP = 0.03;
    public static double kI = 0.00000001;
    public static double kD = 0.00004;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Memory.robotPose);
        start = Memory.robotPose;
        super.reset();
        telemetryData = new TelemetryData(telemetry);

        follower.startTeleopDrive(true);
        gamepad = new GamepadEx(gamepad1);
        gamepadOffset = new GamepadEx(gamepad2);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretPID = new PIDController(kP, kI, kD);

        limeMoving = new LimeMoving(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(limeMoving);

        if (Memory.allianceRed) {
            shooterX = 138; shooterY = 138;
            gateX = 6; gateY = 70;
            end = new Pose(36.5, 38, Math.toRadians(90));
            relocalize = new Pose(4.7, 11.04, Math.toRadians(90));
        } else {
            shooterX = 6; shooterY = 138;
            gateX = 138; gateY = 70;
            end = new Pose(105, 33, Math.toRadians(90));
            relocalize = new Pose(135.8, 9.4, Math.toRadians(90));
        }

        if (!Memory.autoRan) Memory.robotPose = new Pose(72, 72, Math.toRadians(90));

        Park = new Path(new BezierLine(start, end));
        Stay = new Path(new BezierLine(start, start));
        Park.setConstantHeadingInterpolation(Math.toRadians(90));

        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY, !Memory.autoRan);
        intake = new Intake(hardwareMap, () -> follower, shooterX, shooterY);
        shooter.turretOff(false);
        shooter.flywheel(true);
        Memory.autoRan = false;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        setupControls();
    }

    private void setupControls() {

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(intake.collect());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(intake.stop());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(intake.reverse());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(intake.stop());

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new SequentialCommandGroup(
                        intake.open(),
                        new ParallelCommandGroup(intake.collect(), intake.LEDon())
                ));
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 &&
                gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(new ParallelCommandGroup(intake.stop(), intake.close(), intake.LEDoff()));
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 &&
                gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new ParallelCommandGroup(intake.collect(), intake.close(), intake.LEDoff()));

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> start = follower.getPose()),
                        new FollowPathCommand(follower, Park)
                ));

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    follower.startTeleopDrive(true);
                }));

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter.flywheel(true));
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(shooter.flywheel(false));

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.decreaseHoodOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.increaseHoodOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(shooter.decreaseTurretOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(shooter.increaseTurretOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter.OffsetZero());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> follower.setPose(relocalize))
        );

        new Trigger(() -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(() -> {
                    LimeMoving.turretOn = !LimeMoving.turretOn;
                    telemetry.addData("Turret", LimeMoving.turretOn ? "Enabled" : "Disabled");
                    telemetry.update();
                });
    }

    @Override
    public void run() {
        super.run();
        CommandScheduler.getInstance().run();
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        double vx_r = -gamepad1.left_stick_y * multiplier;
        double vy_r = -gamepad1.left_stick_x * multiplier;
        double omega = -gamepad1.right_stick_x * multiplier;

        follower.setTeleOpDrive(vx_r, vy_r, omega, true);
        follower.update();

        LLResult result = limeMoving.getLimelight().getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                LLResultTypes.FiducialResult tag = tags.get(0);
                int tagId = tag.getFiducialId();
                if (tagId == 20 || tagId == 24) {
                    double tx = result.getTx();
                    telemetry.addData("Target TX", tx);
                    telemetry.update();
                }
            }
        }
    }
}
