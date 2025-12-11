package org.firstinspires.ftc.teamcode.teleop;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

import java.util.List;

@Config
@TeleOp
public class TeleopMoving extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private GamepadEx gamepad, gamepadOffset;
    private Intake intake;
    private ShooterMove shooter;
    private Limelight limelight;
    public static double shooterX, shooterY, gateX, gateY;
    private double multiplier = 1;
    private Path Park, Stay;
    private Pose end, start, relocalize;
    List<LynxModule> allHubs;
    private long lastLoopTimeNanos = -1;
    private double maxDecel = 60.0;    // inches/sec^2, TUNE this for your robot
    private double reactionTime = 0.06; // seconds, approx controller/motor response
    private double safeDistance = 20;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Memory.robotPose);
        start = Memory.robotPose;
        super.reset();

        follower.startTeleopDrive(true);
        gamepad = new GamepadEx(gamepad1);
        gamepadOffset = new GamepadEx(gamepad2);

        if (Memory.allianceRed) {
            shooterX = 138;
            shooterY = 138;
            gateX = 6;
            gateY = 70;
            end = new Pose(36.5, 38, Math.toRadians(90));
            relocalize = new Pose(4.7, 11.04, Math.toRadians(90));
        } else {
            shooterX = 6;
            shooterY = 138;
            gateX = 138;
            gateY = 70;
            end = new Pose(105, 33, Math.toRadians(90));
            relocalize = new Pose(135.8, 9.4, Math.toRadians(90));
        }
        if (!Memory.autoRan) {
            Memory.robotPose = new Pose(72, 72, Math.toRadians(90));
        }
        Park = new Path(new BezierLine(start, end));
        Stay = new Path(new BezierLine(start, start));
        Park.setConstantHeadingInterpolation(Math.toRadians(90));
        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY, !Memory.autoRan);
        intake = new Intake(hardwareMap, () -> follower, shooterX, shooterY);
        limelight = new Limelight(hardwareMap, () -> follower);
        shooter.turretOff(false);
        shooter.flywheel(true);
        Memory.autoRan = false;

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> Memory.allianceRed = false)
        );

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(intake.collect());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5).whenActive(intake.stop());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                intake.reverse()
        );

        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                new SequentialCommandGroup(
                        intake.open(),
                        new ParallelCommandGroup(
                                intake.collect(),
                                intake.LEDon()
//                        new InstantCommand(() -> multiplier = 0.1)
                        )
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

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                shooter.turretOff(true)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                shooter.turretOff(false)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> start = follower.getPose()),
                        new FollowPathCommand(follower, Park)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    follower.startTeleopDrive(true);   // restart manual driving
                })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> start = follower.getPose()),
                        new FollowPathCommand(follower, Stay, true)
                )
        );
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                shooter.flywheel(false)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.flywheel(true)
        );



        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                shooter.decreaseHoodOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            shooter.increaseHoodOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                shooter.decreaseTurretOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            shooter.increaseTurretOffset()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                shooter.OffsetZero()
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> follower.setPose(relocalize))
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                limelight.relocalize()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                limelight.norelocalize()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                limelight.fixTurret()
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                limelight.nofixTurret()
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
            double X = pose.getX();
            double Y = pose.getY();
            double heading = pose.getHeading();

            // robot-centric joystick inputs
            double vx_r = -gamepad1.left_stick_y * multiplier;
            double vy_r = -gamepad1.left_stick_x * multiplier;
            double omega = -gamepad1.right_stick_x * multiplier;

            // current field-frame velocity from follower
            double velocityX = follower.getVelocity().getXComponent(); // in/sec
            double velocityY = follower.getVelocity().getYComponent(); // in/sec

            // convert commanded robot-frame => field-frame
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double vx_f = vx_r * cosH - vy_r * sinH;
            double vy_f = vx_r * sinH + vy_r * cosH;

            // vector to gate (field frame)
            double gx = gateX - X;
            double gy = gateY - Y;
            double distToGate = Math.hypot(gx, gy);
            Log.d("Distance to Gate", String.valueOf(distToGate));

            // unit direction toward gate (guard against zero distance)
            double gxHat = 0.0, gyHat = 0.0;
            if (distToGate > 1e-6) {
                gxHat = gx / distToGate;
                gyHat = gy / distToGate;
            }

            // projections (positive = moving toward gate)
            double currentProj = velocityX * gxHat + velocityY * gyHat; // in/sec
            double commandedProj = vx_f * gxHat + vy_f * gyHat;         // in/sec

            // estimate loop dt (for debug / optional use)
            long now = System.nanoTime();
            double dt;
            if (lastLoopTimeNanos < 0) {
                dt = 0.02; // assume 20 ms for first iteration
            } else {
                dt = (now - lastLoopTimeNanos) / 1e9;
                if (dt <= 0) dt = 0.02;
            }
            lastLoopTimeNanos = now;

            // clearance we want to preserve (distance we can travel before hitting safeDistance)
            double distClearance = distToGate - safeDistance;

            // default: allow whole commanded vector
            double vx_f_safe = vx_f;
            double vy_f_safe = vy_f;

            if (distToGate <= safeDistance) {
                // already inside safe margin: remove any commanded toward-gate motion
                if (commandedProj > 0.0) {
                    double remove = commandedProj;
                    vx_f_safe = vx_f - remove * gxHat;
                    vy_f_safe = vy_f - remove * gyHat;
                }
            } else {
                // Predict whether current + commanded motion may cross the safe boundary.
                // Compute allowed net closing speed that would permit stopping before the clearance:
                // stopping_distance = v_net * reactionTime + v_net^2 / (2 * maxDecel)
                // Solve for v_net given stopping_distance = distClearance
                double allowedNetSpeed = 0.0;
                if (distClearance <= 0.0) {
                    allowedNetSpeed = 0.0;
                } else {
                    // quadratic: a*v^2 + b*v - distClearance = 0
                    double a = 1.0 / (2.0 * maxDecel);
                    double b = reactionTime;
                    double c = -distClearance;
                    double disc = b * b - 4.0 * a * c; // should be >= 0
                    if (disc < 0) disc = 0.0;
                    // positive root
                    allowedNetSpeed = (-b + Math.sqrt(disc)) / (2.0 * a);
                    if (allowedNetSpeed < 0.0) allowedNetSpeed = 0.0;
                }

                // compute how much commanded projection is allowed:
                // allowed_commanded_proj = allowedNetSpeed - currentProj
                double allowedCommandedProj = allowedNetSpeed - currentProj;

                // clamp allowedCommandedProj to >= 0 (we don't grant negative commanded allowance)
                if (allowedCommandedProj < 0.0) allowedCommandedProj = 0.0;

                if (commandedProj > allowedCommandedProj) {
                    // need to reduce the commanded toward-gate component
                    double allowedProj = allowedCommandedProj;                // what we may still command
                    double removeProj = commandedProj - allowedProj;         // what to remove from commanded vector
                    if (removeProj < 0) removeProj = 0;

                    vx_f_safe = vx_f - removeProj * gxHat;
                    vy_f_safe = vy_f - removeProj * gyHat;

                    // If currentProj > allowedNetSpeed (we're already too fast toward gate),
                    // the removal above only prevents adding more toward speed. For aggressive braking,
                    // you could command negative projection here (i.e., request reverse) — left to you.
                    if (currentProj > (allowedNetSpeed + 1e-3)) {
                        Log.w("GateSafety", "Already closing faster than safe; consider active braking.");
                        // Optionally: force translational commands to zero so you stop accelerating tangentially:
                        // vx_f_safe = 0.0; vy_f_safe = 0.0;
                    }
                }
            }


            // convert safe field-frame back to robot-frame
            double vx_r_safe = vx_f_safe * cosH + vy_f_safe * sinH;
            double vy_r_safe = -vx_f_safe * sinH + vy_f_safe * cosH;

            // apply final teleop drive (allow rotation unchanged)
            follower.setTeleOpDrive(vx_r_safe, vy_r_safe, omega, true);
        }
        follower.update();

        Park = new Path(new BezierLine(start, end));
        Park.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }
}