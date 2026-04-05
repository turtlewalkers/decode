package org.firstinspires.ftc.teamcode.v2.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.v2.subsystems.Intake;
import org.firstinspires.ftc.teamcode.v2.subsystems.ShooterMove;

import java.util.List;

@Config
@TeleOp(name = "TeleopMoving", group = "V2")
public class TeleopMoving extends CommandOpMode {

    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamepadEx gamepad, gamepadOffset;
    private Intake intake;
    private ShooterMove shooter;

    public static double shooterX, shooterY, gateX, gateY;

    private double multiplier = 1;
    private Path Park, Stay;
    private Pose end, start, relocalize;
    List<LynxModule> allHubs;

    // Gate safety
    private long lastLoopTimeNanos = -1;
    private static final double MAX_DECEL    = 60.0;
    private static final double REACTION_TIME = 0.06;
    private static final double SAFE_DISTANCE = 20.0;

    private GoBildaPinpointDriver pinpoint;

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

        gamepad       = new GamepadEx(gamepad1);
        gamepadOffset = new GamepadEx(gamepad2);

        // Alliance-dependent shooter + gate + park positions (same field as V1)
        if (Memory.allianceRed) {
            shooterX   = 138;
            shooterY   = 138;
            gateX      = 6;
            gateY      = 70;
            end        = new Pose(36.5, 38, Math.toRadians(90));
            relocalize = new Pose(12.315, 8.7159, Math.toRadians(174.392));
        } else {
            shooterX   = 6;
            shooterY   = 138;
            gateX      = 138;
            gateY      = 70;
            end        = new Pose(105, 33, Math.toRadians(90));
            relocalize = new Pose(132.8288, 8.3812, Math.toRadians(6.7524));
        }

        if (!Memory.autoRan) {
            Memory.robotPose = new Pose(72, 72, Math.toRadians(90));
        }

        Park = new Path(new BezierLine(start, end));
        Stay = new Path(new BezierLine(start, start));
        Park.setConstantHeadingInterpolation(Math.toRadians(90));

        intake  = new Intake(hardwareMap, Math.sqrt((shooterX -follower.getPose().getX())*(shooterX -follower.getPose().getX()) + (shooterY -follower.getPose().getY())*(shooterY -follower.getPose().getY())));
        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY);
        register(shooter);

        // Flywheel always on at startup
        shooter.flywheel(true);
        Memory.autoRan = false;

        // ---------------------------------------------------------------
        // Gamepad 1 bindings
        // ---------------------------------------------------------------

        // Left trigger — collect: intake + transfer run, stall stops transfer
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(intake.collectStart());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5
                && gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5)
                .whenActive(intake.collectStop());

        // Right trigger — shoot: latch open, intake + transfer run continuously
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(intake.shootStart());
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5
                && gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5)
                .whenActive(intake.shootStop());

        // DPAD down — reverse intake
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(intake.reverse());
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenReleased(intake.collectStop());

        // A — flywheel on
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter.flywheel(true));

        // B — flywheel off
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(shooter.flywheel(false));

        // Y — park
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    start = follower.getPose();
                    Park = new Path(new BezierLine(start, end));
                    Park.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
                })
        ).whenPressed(new FollowPathCommand(follower, Park));

        // X — cancel auto commands, resume manual drive
        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    follower.startTeleopDrive(true);
                })
        );

        // DPAD up — hold position (Stay path)
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> start = follower.getPose())
        ).whenPressed(new FollowPathCommand(follower, Stay, true));

        // ---------------------------------------------------------------
        // Gamepad 2 bindings (offset tuning)
        // ---------------------------------------------------------------

        // DPAD up/down — hood angle offset
        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.increaseHoodOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(shooter.decreaseHoodOffset());

        // LB/RB — turret angle offset
        gamepadOffset.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(shooter.increaseTurretOffset());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(shooter.decreaseTurretOffset());

        // A — zero all offsets
        gamepadOffset.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter.offsetZero());

        // B — manual relocalize (snap pose to known position)
        gamepadOffset.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> follower.setPose(relocalize))
        );

        // DPAD right (gamepad2) — toggle alliance color
        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> Memory.allianceRed = !Memory.allianceRed)
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetryData.addData("Pose",  Memory.robotPose);
        telemetryData.addData("Start", start);
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
            double X       = pose.getX();
            double Y       = pose.getY();
            double heading = pose.getHeading();
            double cosH    = Math.cos(heading);
            double sinH    = Math.sin(heading);

            double vx_r = -gamepad1.left_stick_y * multiplier;
            double vy_r = -gamepad1.left_stick_x * multiplier;
            double omega = -gamepad1.right_stick_x * multiplier;

            // Robot-frame → field-frame
            double vx_f = vx_r * cosH - vy_r * sinH;
            double vy_f = vx_r * sinH + vy_r * cosH;

            // Gate safety — prevent driving through the gate
            double gx = gateX - X;
            double gy = gateY - Y;
            double distToGate = Math.hypot(gx, gy);

            double gxHat = 0.0, gyHat = 0.0;
            if (distToGate > 1e-6) {
                gxHat = gx / distToGate;
                gyHat = gy / distToGate;
            }

            double velocityX = follower.getVelocity().getXComponent();
            double velocityY = follower.getVelocity().getYComponent();
            double currentProj   = velocityX * gxHat + velocityY * gyHat;
            double commandedProj = vx_f * gxHat + vy_f * gyHat;

            long now = System.nanoTime();
            double dt;
            if (lastLoopTimeNanos < 0) {
                dt = 0.02;
            } else {
                dt = (now - lastLoopTimeNanos) / 1e9;
                if (dt <= 0) dt = 0.02;
            }
            lastLoopTimeNanos = now;

            double distClearance = distToGate - SAFE_DISTANCE;
            double vx_f_safe = vx_f;
            double vy_f_safe = vy_f;

            if (distToGate <= SAFE_DISTANCE) {
                if (commandedProj > 0.0) {
                    vx_f_safe = vx_f - commandedProj * gxHat;
                    vy_f_safe = vy_f - commandedProj * gyHat;
                }
            } else {
                double allowedNetSpeed;
                if (distClearance <= 0.0) {
                    allowedNetSpeed = 0.0;
                } else {
                    double a    = 1.0 / (2.0 * MAX_DECEL);
                    double b    = REACTION_TIME;
                    double c    = -distClearance;
                    double disc = b * b - 4.0 * a * c;
                    if (disc < 0) disc = 0.0;
                    allowedNetSpeed = (-b + Math.sqrt(disc)) / (2.0 * a);
                    if (allowedNetSpeed < 0.0) allowedNetSpeed = 0.0;
                }

                double allowedCommandedProj = Math.max(0.0, allowedNetSpeed - currentProj);

                if (commandedProj > allowedCommandedProj) {
                    double removeProj = commandedProj - allowedCommandedProj;
                    vx_f_safe = vx_f - removeProj * gxHat;
                    vy_f_safe = vy_f - removeProj * gyHat;
                    Log.d("GateSafety", String.format("clamped proj: %.2f → %.2f", commandedProj, allowedCommandedProj));
                }
            }

            // Field-frame → robot-frame for PedroPathing
            double vx_r_safe =  vx_f_safe * cosH + vy_f_safe * sinH;
            double vy_r_safe = -vx_f_safe * sinH + vy_f_safe * cosH;

            follower.setTeleOpDrive(vx_r_safe, vy_r_safe, omega, true);
        }
        follower.update();

        telemetryData.addData("X",           follower.getPose().getX());
        telemetryData.addData("Y",           follower.getPose().getY());
        telemetryData.addData("Heading",     Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("TurretDeg",   ShooterMove.turretPosDeg);
        telemetryData.addData("FlywheelRPM", shooter.getFlywheelRpm());
        telemetryData.addData("Alliance",    Memory.allianceRed ? "RED" : "BLUE");

        double vx   = follower.getVelocity().getXComponent();
        double vy   = follower.getVelocity().getYComponent();
        double speed = Math.hypot(vx, vy);
        telemetryData.addData("Speed (in/s)", speed);
        telemetryData.addData("VX (in/s)",    vx);
        telemetryData.addData("VY (in/s)",    vy);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Speed (in/s)",   speed);
        packet.put("VX (in/s)",      vx);
        packet.put("VY (in/s)",      vy);
        packet.put("TurretDeg",      ShooterMove.turretPosDeg);
        packet.put("ServoPos", shooter.getServoPos());
        packet.put("Heading", Math.toDegrees(follower.getPose().getHeading()));
        packet.put("FlywheelRPM",    shooter.getFlywheelRpm());
        dashboard.sendTelemetryPacket(packet);

        telemetryData.update();
    }
}
