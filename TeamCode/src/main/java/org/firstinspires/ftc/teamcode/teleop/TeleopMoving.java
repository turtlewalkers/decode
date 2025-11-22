package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

@Config
@TeleOp
public class TeleopMoving extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private GamepadEx gamepad, gamepadOffset;
    private Intake intake;
    private ShooterMove shooter;
    public static double shooterX, shooterY;
    private double multiplier = 1;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Memory.robotPose);
        super.reset();

        follower.startTeleopDrive(true);
        gamepad = new GamepadEx(gamepad1);
        gamepadOffset = new GamepadEx(gamepad2);
        intake = new Intake(hardwareMap);

        if (Memory.allianceRed) {
            shooterX = 138;
            shooterY = 138;
        } else {
            shooterX = 6;
            shooterY = 138;
        }
        if (!Memory.autoRan) {
            Memory.robotPose = new Pose(72, 72, Math.toRadians(90));
        }
        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY, !Memory.autoRan);
        shooter.turretOff(false);
        Memory.autoRan = false;

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(intake.collect());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5).whenActive(intake.stop());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                intake.reverse()
        );

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                new ParallelCommandGroup(
                        intake.collect(),
                        intake.open(),
                        intake.LEDon()
//                        new InstantCommand(() -> multiplier = 0.1)
                )
        );

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5).whenActive(
                new ParallelCommandGroup(
                        intake.stop(),
                        intake.close(),
                        intake.LEDoff()
//                        new InstantCommand(() -> multiplier = 1)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.flywheel(true)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                shooter.flywheel(false)
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                shooter.decreaseHoodOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            shooter.increaseHoodOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                shooter.decreaseTurretOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            shooter.increaseTurretOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.OffsetZero()
        );
    }

    @Override
    public void run() {
        super.run();

        follower.setTeleOpDrive(-gamepad1.left_stick_y * multiplier, -gamepad1.left_stick_x * multiplier, -gamepad1.right_stick_x * multiplier, true);
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }
}