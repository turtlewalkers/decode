package org.firstinspires.ftc.teamcode.v2.teleop;

import static org.firstinspires.ftc.teamcode.v2.subsystems.Intake.SHOOT_SPEED;

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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.v2.shooter.Shooter;
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

    public static boolean DEBUG_MODE = false;  // Dashboard-tunable: false = match mode, true = full telemetry
    public static double OMEGA_SCALE = 0.7; //rotation scaling lower to give translation more priority
    public static double MATCH_DURATION = 120.0;   // seconds
    public static double ENDGAME_THRESHOLD = 30.0;   // last N seconds

    // Smart relocalization
    public static double RELOC_THRESHOLD = 25.0;     // inches from boundary to trigger
    public static double RELOC_HEADING_TOL = 45.0;   // degrees tolerance for front/back detection
    private static final double RAMP_Y_MIN = 70.0;
    private static final double RAMP_Y_MAX = 118.0;

    private double multiplier = 1;
    private Path Park, Stay;
    private Pose end, start;
    List<LynxModule> allHubs;

    // Gate safety
    private long lastLoopTimeNanos = -1;
    private static final double MAX_DECEL    = 60.0;
    private static final double REACTION_TIME = 0.06;
    private static final double SAFE_DISTANCE = 20.0;

    private GoBildaPinpointDriver pinpoint;

    private long startTimeMs = 0;
    private ServoEx tip;
    private boolean tipDeployed = false;

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

        tip = new ServoEx(hardwareMap, "tip");
      /*  PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);
        ((PwmControl) hardwareMap.get(Servo.class, "tip")).setPwmRange(axonRange); */

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
            gateY      = 72;
            end        = new Pose(36.5, 38, Math.toRadians(90));
        } else {
            shooterX   = 6;
            shooterY   = 138;
            gateX      = 138;
            gateY      = 72;
            end        = new Pose(105, 33, Math.toRadians(90));
        }

        if (!Memory.autoRan) {
            Memory.robotPose = new Pose(72, 72, Math.toRadians(90));
        }

        Park = new Path(new BezierLine(start, end));
        Stay = new Path(new BezierLine(start, start));
        Park.setConstantHeadingInterpolation(Math.toRadians(90));

        intake  = new Intake(hardwareMap, () -> follower, shooterX, shooterY);
        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY);
        register(shooter);

        // Flywheel always on at startup
        shooter.flywheel(true);
       // shooter.setPredictiveAim(true);
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

        // RB — Toggle SWM
        //gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(shooter.SWMToggle());

        // LB — hold to disable shoot-while-moving (faster loops for gate cycles)
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(shooter.SWMoff());
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(shooter.SWMon());

        // B — flywheel off
        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(shooter.flywheel(false));

        // Y — park
       /* gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    start = follower.getPose();
                    Park = new Path(new BezierLine(start, end));
                    Park.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
                })
        ).whenPressed(new FollowPathCommand(follower, Park));*/

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

        // B — smart relocalize (auto-detect nearest wall/ramp)
        // Temporarily change to gamepad Y and remove park.
        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> smartRelocalize())
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> smartRelocalize())
        );

        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(shooter.increaseRpmScalar());
        gamepadOffset.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(shooter.decreaseRpmScalar());

        // DPAD right (gamepad2) — toggle alliance color
        gamepadOffset.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new InstantCommand(() -> Memory.allianceRed = !Memory.allianceRed),
                new InstantCommand(() -> {
                    Memory.allianceRed = !Memory.allianceRed;
                    if (Memory.allianceRed) {
                        shooterX = 138;
                        shooterY = 138;
                        gateX = 6;
                        gateY = 70;
                    } else {
                        shooterX = 6;
                        shooterY = 138;
                        gateX = 138;
                        gateY = 70;
                    }
                    intake.setShooterPos(shooterX,  shooterY);
                    shooter.setShooterPos(shooterX, shooterY);
                })
        );
        // X (gamepad2) — deploy tip servo (endgame only, last ENDGAME_THRESHOLD seconds)
        gamepadOffset.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                  double elapsed = (System.currentTimeMillis() - startTimeMs) / 1000.0;
                    double remainingSec = MATCH_DURATION - elapsed;
                    if (startTimeMs > 0 && remainingSec <= ENDGAME_THRESHOLD) {
                        tipDeployed = true;
                        tip.set(tipDeployed ? 1.0 : 0.0);
                    }
                   // tip.set(tipDeployed ? 1.0 : 0.0);
                })
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
     //   super.run();
//        if (Memory.allianceRed) {
//            if (shooterX == 138) {
//                shooterX = 138;
//                shooterY = 138;
//                gateX = 6;
//                gateY = 70;
//                end = new Pose(36.5, 38, Math.toRadians(90));
//            }
//        } else {
//            if (shooterX == 6) {
//                shooterX = 6;
//                shooterY = 6;
//                gateX = 138;
//                gateY = 70;
//                end = new Pose(107.5, 38, Math.toRadians(270));
//            }
//        }270
//        shooter = new ShooterMove(hardwareMap, () -> follower, shooterX, shooterY);

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
            double omega = -gamepad1.right_stick_x * multiplier * OMEGA_SCALE;

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
                    if (DEBUG_MODE) {
                        Log.d("GateSafety", String.format("clamped proj: %.2f → %.2f", commandedProj, allowedCommandedProj));
                    }
                }
            }

            // Field-frame → robot-frame for PedroPathing
            double vx_r_safe =  vx_f_safe * cosH + vy_f_safe * sinH;
            double vy_r_safe = -vx_f_safe * sinH + vy_f_safe * cosH;

            follower.setTeleOpDrive(vx_r_safe * SHOOT_SPEED, vy_r_safe * SHOOT_SPEED, omega * SHOOT_SPEED, true);
        }
        follower.update();

        super.run();

        // Match timer
        if (startTimeMs == 0) {
            startTimeMs = System.currentTimeMillis();
        }
        Memory.debugMode = DEBUG_MODE;
        double elapsed = (System.currentTimeMillis() - startTimeMs) / 1000.0;
        double remainingSec = Math.max(0, MATCH_DURATION - elapsed);

        // Always show: minimal driver hub telemetry
        telemetryData.addData("X",           follower.getPose().getX());
        telemetryData.addData("Y",           follower.getPose().getY());
        telemetryData.addData("Heading",     Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.addData("Alliance",    Memory.allianceRed ? "RED" : "BLUE");
        telemetryData.addData("Balls",       intake.getBallCount());
        telemetryData.addData("Time",        String.format("%.1f", remainingSec));
        telemetryData.addData("Tip",         tipDeployed ? "DEPLOYED" : "STOWED");

        if (DEBUG_MODE) {
            telemetryData.addData("TurretDeg",   ShooterMove.turretPosDeg);
            telemetryData.addData("FlywheelRPM", shooter.getFlywheelRpm());

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
            packet.put("TurretTargetDeg",ShooterMove.turretTargetDeg);
            packet.put("RobotHeadingDeg",   ShooterMove.robotHeadingDeg);
            packet.put("ServoPos", shooter.getServoPos());
            packet.put("Heading", Math.toDegrees(follower.getPose().getHeading()));
            packet.put("BatteryVoltage",  ShooterMove.batteryVoltage);
            packet.put("X", follower.getPose().getX());
            packet.put("Y", follower.getPose().getY());
            packet.put("LoopTimeMs", ShooterMove.loopTimeMs);
            packet.put("OmegaDegPerSec", ShooterMove.omegaDegPerSec);
            packet.put("ServoLagCompDeg", ShooterMove.servoLagCompDeg);
            packet.put("FlywheelRPM",    shooter.getFlywheelRpm());
            dashboard.sendTelemetryPacket(packet);
        }

        telemetryData.update();
    }

    private void smartRelocalize() {
        Pose current = follower.getPose();
        double x = current.getX();
        double y = current.getY();
        double headingDeg = Math.toDegrees(current.getHeading()) % 360;
        if (headingDeg < 0) headingDeg += 360;

        double newX = x;
        double newY = y;
        boolean isRampZone = (y >= RAMP_Y_MIN && y <= RAMP_Y_MAX);

        // --- Y-axis correction ---
        if (y > 144 - RELOC_THRESHOLD) {
            // Back wall
            if (isFacing(headingDeg, 90, RELOC_HEADING_TOL)) {
                newY = 132.8;
            } else if (isFacing(headingDeg, 270, RELOC_HEADING_TOL)) {
                newY = 136.5;
            }
        } else if (y < RELOC_THRESHOLD) {
            // Audience wall
            if (isFacing(headingDeg, 270, RELOC_HEADING_TOL)) {
                newY = 13.14;
            } else if (isFacing(headingDeg, 90, RELOC_HEADING_TOL)) {
                newY = 10.1;
            }
        }

        // --- X-axis correction (both sides) ---
        if (x < RELOC_THRESHOLD) {
            // Blue side
            if (isRampZone) {
                if (isFacing(headingDeg, 180, RELOC_HEADING_TOL)) {
                    newX = 17.8;
                } else if (isFacing(headingDeg, 0, RELOC_HEADING_TOL)) {
                    newX = 14.2;
                }
            } else {
                if (isFacing(headingDeg, 180, RELOC_HEADING_TOL)) {
                    newX = 11.5;
                } else if (isFacing(headingDeg, 0, RELOC_HEADING_TOL)) {
                    newX = 8.3;
                }
            }
        } else if (x > 144 - RELOC_THRESHOLD) {
            // Red side
            if (isRampZone) {
                if (isFacing(headingDeg, 0, RELOC_HEADING_TOL)) {
                    newX = 126.3;
                } else if (isFacing(headingDeg, 180, RELOC_HEADING_TOL)) {
                    newX = 129.4;
                }
            } else {
                if (isFacing(headingDeg, 0, RELOC_HEADING_TOL)) {
                    newX = 133.6;
                } else if (isFacing(headingDeg, 180, RELOC_HEADING_TOL)) {
                    newX = 136.5;
                }
            }
        }

        if (newX != x || newY != y) {
            double newHeading = current.getHeading();

            // Corner — use precise measured poses at audience-side corners
            if (newX != x && newY != y && y < RELOC_THRESHOLD) {
                if (Memory.allianceRed && x < RELOC_THRESHOLD) {
                    newX = 12.315;
                    newY = 8.7159;
                    newHeading = Math.toRadians(174.392);
                } else if (!Memory.allianceRed && x > 144 - RELOC_THRESHOLD) {
                    newX = 132.8288;
                    newY = 8.3812;
                    newHeading = Math.toRadians(6.7524);
                }
            }

            follower.setPose(new Pose(newX, newY, newHeading));
        }
    }

    private boolean isFacing(double headingDeg, double targetDeg, double tolerance) {
        double diff = Math.abs(headingDeg - targetDeg);
        if (diff > 180) diff = 360 - diff;
        return diff <= tolerance;
    }
}
