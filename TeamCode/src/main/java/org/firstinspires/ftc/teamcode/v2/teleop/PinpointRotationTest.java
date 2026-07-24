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
 * Pinpoint pod offset diagnostic — GoBilda Step 10 rotation test.
 *
 * Spins the robot in place at a fixed speed and tracks X/Y drift.
 * If pod offsets are correct, X and Y should stay near the start position.
 *
 * Controls:
 * - A — start spinning at 0.5 power
 * - B — stop spinning
 * - X — reset pose and drift counters
 *
 * What to watch:
 * - Dashboard field view shows robot dot + trail — should stay near (72,72)
 * - Dashboard graphs show DriftX and DriftY over time
 * - Telemetry auto-diagnoses: "Y drift high → strafePodX may be wrong" etc.
 * - ROTATION_SPEED is Dashboard-tunable if you want to try different speeds
 */
@Config
@TeleOp(name = "PinpointRotationTest", group = "V2")
public class PinpointRotationTest extends OpMode {

    public static double ROTATION_SPEED = 0.5;

    private static final double START_X = 72.0;
    private static final double START_Y = 72.0;
    private static final double START_HEADING = Math.toRadians(90);

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final String TAG = "RotationTest";

    private boolean spinning = false;
    private double maxDriftX = 0;
    private double maxDriftY = 0;
    private int rotationCount = 0;
    private double lastHeading = START_HEADING;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        follower.startTeleopDrive(true);

        telemetry.addLine("PinpointRotationTest ready");
        telemetry.addLine("A = spin | B = stop | X = reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) spinning = true;
        if (gamepad1.b) spinning = false;
        if (gamepad1.x) {
            follower.setPose(new Pose(START_X, START_Y, START_HEADING));
            maxDriftX = 0;
            maxDriftY = 0;
            rotationCount = 0;
            lastHeading = START_HEADING;
            spinning = false;
        }

        double omega = spinning ? ROTATION_SPEED : 0;
        follower.setTeleOpDrive(0, 0, omega, true);
        follower.update();

        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double driftX = Math.abs(x - START_X);
        double driftY = Math.abs(y - START_Y);
        maxDriftX = Math.max(maxDriftX, driftX);
        maxDriftY = Math.max(maxDriftY, driftY);

        // Count full rotations
        double headingDelta = heading - lastHeading;
        if (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
        if (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
        lastHeading = heading;
        // Use totalHeading for rotation count
        double totalHeading = follower.getTotalHeading();
        rotationCount = (int) Math.abs(totalHeading / (2 * Math.PI));

        // Raw pinpoint values
        pinpoint.update();
        Pose2D rawPose = pinpoint.getPosition();
        double rawX = rawPose.getX(DistanceUnit.INCH);
        double rawY = rawPose.getY(DistanceUnit.INCH);
        double rawH = rawPose.getHeading(AngleUnit.DEGREES);

        // Telemetry
        telemetry.addData("Status", spinning ? "SPINNING" : "STOPPED");
        telemetry.addData("Rotations", rotationCount);
        telemetry.addLine("--- Pedro ---");
        telemetry.addData("X", "%.2f", x);
        telemetry.addData("Y", "%.2f", y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.addLine("--- Raw Pinpoint ---");
        telemetry.addData("Raw X", "%.2f", rawX);
        telemetry.addData("Raw Y", "%.2f", rawY);
        telemetry.addData("Raw Heading", "%.1f", rawH);
        telemetry.addLine("--- Drift from start ---");
        telemetry.addData("Drift X", "%.2f in", driftX);
        telemetry.addData("Drift Y", "%.2f in", driftY);
        telemetry.addData("Max Drift X", "%.2f in", maxDriftX);
        telemetry.addData("Max Drift Y", "%.2f in", maxDriftY);
        telemetry.addLine("--- Diagnosis ---");
        if (maxDriftX > 4) telemetry.addLine("!! X drift high → forwardPodY may be wrong");
        if (maxDriftY > 4) telemetry.addLine("!! Y drift high → strafePodX may be wrong");
        if (maxDriftX <= 4 && maxDriftY <= 4) telemetry.addLine("Pod offsets look good");
        telemetry.addLine("A=spin  B=stop  X=reset");
        telemetry.update();

        // FTC Dashboard graphs + field overlay
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X", x);
        packet.put("Y", y);
        packet.put("DriftX", driftX);
        packet.put("DriftY", driftY);
        packet.put("MaxDriftX", maxDriftX);
        packet.put("MaxDriftY", maxDriftY);
        packet.put("HeadingDeg", Math.toDegrees(heading));
        packet.put("Rotations", rotationCount);
        packet.put("RawX", rawX);
        packet.put("RawY", rawY);
        packet.put("RawHeading", rawH);
        drawRobot(packet, x, y, heading);
        dashboard.sendTelemetryPacket(packet);

        Log.d(TAG, String.format("rot=%d x=%.3f y=%.3f h=%.1f dX=%.3f dY=%.3f rawX=%.3f rawY=%.3f rawH=%.1f",
                rotationCount, x, y, Math.toDegrees(heading), driftX, driftY, rawX, rawY, rawH));
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
