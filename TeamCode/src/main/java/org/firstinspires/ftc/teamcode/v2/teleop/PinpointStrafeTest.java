package org.firstinspires.ftc.teamcode.v2.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pinpoint strafe drift test.
 *
 * Auto-oscillates left and right 24 inches (robot frame).
 * Uses setTeleOpDrive with fieldCentric=true so robot-strafe input
 * always moves the robot sideways.
 * Tracks cross-axis drift which should be ~0 during pure strafing.
 *
 * Controls:
 *   A — start auto-oscillation
 *   B — stop
 *   X — reset pose and drift counters
 */
@Config
@TeleOp(name = "PinpointStrafeTest", group = "V2")
public class PinpointStrafeTest extends OpMode {

    public static double STRAFE_SPEED = 0.3;
    public static double OSCILLATE_DISTANCE = 24.0;
    public static double SLOW_ZONE = 6.0;
    public static int MAX_LAPS = 20;

    private static final double START_X = 72.0;
    private static final double START_Y = 72.0;
    private static final double START_HEADING = Math.toRadians(90);

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final String TAG = "StrafeTest";

    private boolean running = false;
    private boolean goingLeft = true;
    private double maxDriftCross = 0;
    private int lapCount = 0;
    private double turnpointX, turnpointY;

    @Override
    public void init() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        follower.startTeleopDrive(true);

        turnpointX = START_X;
        turnpointY = START_Y;

        telemetry.addLine("PinpointStrafeTest ready");
        telemetry.addLine("A=start  B=stop  X=reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if (!running) {
                Pose p = follower.getPose();
                turnpointX = p.getX();
                turnpointY = p.getY();
                goingLeft = true;
            }
            running = true;
        }
        if (gamepad1.b) running = false;
        if (gamepad1.x) {
            follower.setPose(new Pose(START_X, START_Y, START_HEADING));
            maxDriftCross = 0;
            lapCount = 0;
            goingLeft = true;
            running = false;
            turnpointX = START_X;
            turnpointY = START_Y;
        }

        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();

        double distTraveled = Math.hypot(x - turnpointX, y - turnpointY);

        if (running) {
            if (lapCount >= MAX_LAPS) {
                running = false;
            } else if (distTraveled >= OSCILLATE_DISTANCE) {
                goingLeft = !goingLeft;
                turnpointX = x;
                turnpointY = y;
                lapCount++;
                distTraveled = 0;
            }

            double distToEnd = OSCILLATE_DISTANCE - distTraveled;
            double speedScale = Math.min(1.0, distToEnd / SLOW_ZONE);
            double speed = STRAFE_SPEED * (0.15 + 0.85 * speedScale);

            // Robot-frame strafe via fieldCentric=true
            double strafe = goingLeft ? speed : -speed;
            follower.setTeleOpDrive(0, strafe, 0, true);
        } else {
            follower.setTeleOpDrive(0, 0, 0, true);
        }
        follower.update();

        pose = follower.getPose();
        x = pose.getX();
        y = pose.getY();
        double heading = pose.getHeading();

        // Cross-axis drift: perpendicular to travel direction
        // At heading 90°, strafe = X axis, so cross-axis = Y drift
        double driftCross = Math.abs(y - START_Y);
        double distMain = x - START_X;
        maxDriftCross = Math.max(maxDriftCross, driftCross);

        // Raw pinpoint values
        pinpoint.update();
        Pose2D rawPose = pinpoint.getPosition();
        double rawX = rawPose.getX(DistanceUnit.INCH);
        double rawY = rawPose.getY(DistanceUnit.INCH);
        double rawH = rawPose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("Status", running ? (goingLeft ? "LEFT" : "RIGHT") : "STOPPED");
        telemetry.addData("Laps", lapCount);
        telemetry.addData("Dist from turnpoint", "%.1f", distTraveled);
        telemetry.addLine("--- Pedro ---");
        telemetry.addData("X", "%.2f", x);
        telemetry.addData("Y", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addLine("--- Raw Pinpoint ---");
        telemetry.addData("Raw X", "%.2f", rawX);
        telemetry.addData("Raw Y", "%.2f", rawY);
        telemetry.addData("Raw Heading", "%.1f", rawH);
        telemetry.addLine("--- Drift ---");
        telemetry.addData("Cross-axis drift", "%.2f in", driftCross);
        telemetry.addData("Main-axis distance", "%.2f in", distMain);
        telemetry.addData("Max cross drift", "%.2f in", maxDriftCross);
        telemetry.addLine("A=start  B=stop  X=reset");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", x);
        packet.put("Y", y);
        packet.put("DriftCross", driftCross);
        packet.put("DistMain", distMain);
        packet.put("MaxDriftCross", maxDriftCross);
        packet.put("HeadingDeg", Math.toDegrees(heading));
        packet.put("RawX", rawX);
        packet.put("RawY", rawY);
        packet.put("RawHeading", rawH);
        packet.put("Laps", lapCount);
        drawRobot(packet, x, y, heading);
        dashboard.sendTelemetryPacket(packet);

        Log.d(TAG, String.format("lap=%d x=%.3f y=%.3f h=%.1f dCross=%.3f distMain=%.3f rawX=%.3f rawY=%.3f rawH=%.1f",
                lapCount, x, y, Math.toDegrees(heading), driftCross, distMain, rawX, rawY, rawH));
    }

    private static void drawRobot(TelemetryPacket packet, double x, double y, double heading) {
        Canvas canvas = packet.fieldOverlay();
        double robotWidth = 18;
        double robotLength = 18;

        canvas.setStroke("blue");
        canvas.strokeRect(x - robotLength / 2, y - robotWidth / 2, robotLength, robotWidth);

        double arrowLength = 10;
        double x2 = x + arrowLength * Math.cos(heading);
        double y2 = y + arrowLength * Math.sin(heading);
        canvas.setStroke("red");
        canvas.strokeLine(x, y, x2, y2);

        canvas.setFill("black");
        canvas.fillCircle(x, y, 1);
    }
}
