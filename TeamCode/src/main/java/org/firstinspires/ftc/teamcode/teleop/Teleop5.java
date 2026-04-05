package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.ShooterMove;

import java.util.List;

@Config
@TeleOp(name = "Teleop5", group = "V2")
public class Teleop5 extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private GamepadEx gamepad, gamepadOffset;
    private Intake intake;
    private ShooterMove shooter;
    public static double shooterX = 138, shooterY = 138;  // tune per alliance
    private double multiplier = 1;
    private Pose start;
    List<LynxModule> allHubs;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx lf, rf, lb, rb;

    @Override
    public void reset() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }

    @Override
    public void initialize() {
        super.reset();
        start = Memory.robotPose;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        follower.startTeleopDrive(true);

        gamepad = new GamepadEx(gamepad1);
        gamepadOffset = new GamepadEx(gamepad2);

        intake = new Intake(hardwareMap, Math.sqrt((shooterX - follower.getPose().getX())*(shooterX - follower.getPose().getX()) + (shooterY - follower.getPose().getY())*(shooterY - follower.getPose().getY())));
        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY);
        register(shooter);

        // Motor order from Mecanum.java:60 — Arrays.asList(leftFront, leftRear, rightFront, rightRear)
        // Source: com.pedropathing/ftc/2.1.0-SNAPSHOT (ftc-2.1.0-SNAPSHOT-sources.jar)
        List<DcMotorEx> driveMotors = ((Mecanum) follower.getDrivetrain()).getMotors();
        lf = driveMotors.get(0);  // leftFront
        lb = driveMotors.get(1);  // leftRear
        rf = driveMotors.get(2);  // rightFront
        rb = driveMotors.get(3);  // rightRear

        // Left trigger — collect mode: intake + transfer run, stall stops transfer
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(intake.collectStart());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5
                && gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(intake.collectStop());

        // Right trigger — shoot mode: latch open, intake + transfer run continuously
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(intake.shootStart());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5
                && gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5)
                .whenActive(intake.shootStop());

        // DPAD down — reverse intake
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(intake.reverse());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(intake.collectStop());

        // Y — toggle flywheel on/off manually (set MANUAL_RPM on Dashboard to test at fixed RPM)
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(shooter.flywheelToggle());

        // X — cancel all commands, resume manual drive
        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    follower.startTeleopDrive(true);
                })
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        Pose pose = follower.getPose();
        if (pose != null) {
            start = pose;
            double vx_r = -gamepad1.left_stick_y * multiplier;
            double vy_r = -gamepad1.left_stick_x * multiplier;
            double omega = -gamepad1.right_stick_x * multiplier;
            follower.setTeleOpDrive(vx_r, vy_r, omega, true);
        }
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("Flywheel RPM", shooter.getFlywheelRpm());
        telemetryData.addData("LF current (A)", lf.getCurrent(CurrentUnit.AMPS));
        telemetryData.addData("RF current (A)", rf.getCurrent(CurrentUnit.AMPS));
        telemetryData.addData("LB current (A)", lb.getCurrent(CurrentUnit.AMPS));
        telemetryData.addData("RB current (A)", rb.getCurrent(CurrentUnit.AMPS));
        telemetryData.update();
    }
}
