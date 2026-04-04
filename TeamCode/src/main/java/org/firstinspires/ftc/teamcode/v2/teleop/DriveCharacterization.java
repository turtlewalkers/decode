package org.firstinspires.ftc.teamcode.v2.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;

import java.util.List;

/**
 * Drive characterization opmode.
 *
 * Driver drives naturally using left/right sticks (same controls as TeleopMoving).
 * Collects live speed, velocity, and acceleration data from the follower odometry.
 * Automatically calculates turret tracking margin at worst-case shooting distances.
 *
 * === HOW TO USE ===
 * 1. Place robot anywhere on field (near back launch zone for realistic data)
 * 2. Run opmode, drive naturally at match speed for 30–60 seconds
 * 3. Push B to reset peak stats and run another pass if needed
 * 4. Record "Peak Speed (in/s)" and "Tracking margin @ 94in" from Dashboard
 *
 * === TURRET TRACKING MARGIN ===
 * Margin = (turret max angular rate) / (required angular rate at distance)
 * Required rate = peak_speed / distance * 57.3 deg/rad  (worst case: pure perpendicular motion)
 * Turret max = 119 deg/s  (confirmed: Axon MAX 167 deg/s servo ÷ 1.408x gear ratio @ 7.4V)
 *
 * Target margins:
 *   @ 94" (back zone apex — worst case): > 1.3x
 *   @ 42" (near goal corner — closest): > 3.0x
 *
 * === CONTROLS ===
 * Left stick Y / Right stick X — field-centric drive (same as TeleopMoving)
 * B            — reset all peak stats
 * DPAD UP/DOWN — increase/decrease drive power multiplier by 0.1
 */
@Config
@TeleOp(name = "DriveCharacterization", group = "V2")
public class DriveCharacterization extends OpMode {

    // Turret max angular rate at 7.4V (Axon MAX 167 deg/s ÷ 1.408x gear ratio)
    public static final double TURRET_MAX_DEG_PER_SEC = 119.0;

    // Worst-case back zone shooting distance — field center apex (72,72) to goal (138,138)
    public static final double DIST_WORST_IN = 94.0;
    // Near corner shooting distance — (108,108) to goal (138,138)
    public static final double DIST_NEAR_IN  = 42.0;

    public static double POWER_MULTIPLIER = 1.0;

    private FtcDashboard dashboard;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private List<LynxModule> allHubs;

    private ElapsedTime loopTimer;
    private double lastSpeed    = 0.0;
    private double lastLoopTime = 0.02;

    // Peak stats — persist until B is pressed
    private double peakSpeed    = 0.0;
    private double peakAccel    = 0.0;
    private double peakDecel    = 0.0;

    private boolean lastB = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        loopTimer = new ElapsedTime();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Memory.robotPose);
        follower.startTeleopDrive(true);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Ready — drive naturally at match speed");
        telemetry.addData("B", "reset peak stats");
        telemetry.addData("DPAD UP/DOWN", "adjust power multiplier");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Reset peaks on B press
        boolean bNow = gamepad1.b;
        if (bNow && !lastB) {
            peakSpeed = 0.0;
            peakAccel = 0.0;
            peakDecel = 0.0;
        }
        lastB = bNow;

        // Power multiplier adjustment
        if (gamepad1.dpad_up)   POWER_MULTIPLIER = Math.min(1.0, POWER_MULTIPLIER + 0.1);
        if (gamepad1.dpad_down) POWER_MULTIPLIER = Math.max(0.0, POWER_MULTIPLIER - 0.1);

        // Drive — same field-centric controls as TeleopMoving
        Pose pose = follower.getPose();
        if (pose != null) {
            double vx_r = -gamepad1.left_stick_y  * POWER_MULTIPLIER;
            double vy_r = -gamepad1.left_stick_x  * POWER_MULTIPLIER;
            double omega = -gamepad1.right_stick_x * POWER_MULTIPLIER;
            follower.setTeleOpDrive(vx_r, vy_r, omega, true);
        }
        follower.update();

        // Measure speed and acceleration
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.02;

        double vx    = follower.getVelocity().getXComponent();
        double vy    = follower.getVelocity().getYComponent();
        double speed = Math.hypot(vx, vy);
        double accel = (speed - lastSpeed) / dt;

        lastSpeed    = speed;
        lastLoopTime = dt;

        // Update peaks
        if (speed > peakSpeed) peakSpeed = speed;
        if (accel > peakAccel) peakAccel = accel;   // peak acceleration
        if (accel < -peakDecel) peakDecel = -accel; // peak deceleration (positive magnitude)

        // Tracking margin: turret angular rate headroom at each reference distance
        // Required rate = peak_speed / distance * (180/PI)   (pure perpendicular worst case)
        double margin94 = 0.0, margin42 = 0.0;
        if (peakSpeed > 0.1) {
            double reqRate94 = peakSpeed / DIST_WORST_IN * Math.toDegrees(1.0);
            double reqRate42 = peakSpeed / DIST_NEAR_IN  * Math.toDegrees(1.0);
            margin94 = TURRET_MAX_DEG_PER_SEC / reqRate94;
            margin42 = TURRET_MAX_DEG_PER_SEC / reqRate42;
        }

        // Driver hub telemetry
        telemetry.addData("Speed (in/s)",          "%.1f", speed);
        telemetry.addData("VX (in/s)",             "%.1f", vx);
        telemetry.addData("VY (in/s)",             "%.1f", vy);
        telemetry.addData("Accel (in/s²)",         "%.1f", accel);
        telemetry.addData("---", "");
        telemetry.addData("Peak Speed (in/s)",     "%.1f", peakSpeed);
        telemetry.addData("Peak Accel (in/s²)",    "%.1f", peakAccel);
        telemetry.addData("Peak Decel (in/s²)",    "%.1f", peakDecel);
        telemetry.addData("Tracking margin @ 94in", "%.2fx  (need >1.3x)", margin94);
        telemetry.addData("Tracking margin @ 42in", "%.2fx  (need >3.0x)", margin42);
        telemetry.addData("Power multiplier",      "%.1f", POWER_MULTIPLIER);
        telemetry.update();

        // Dashboard packet — graphable over time
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Speed (in/s)",          speed);
        packet.put("VX (in/s)",             vx);
        packet.put("VY (in/s)",             vy);
        packet.put("Accel (in/s²)",         accel);
        packet.put("Peak Speed (in/s)",     peakSpeed);
        packet.put("Peak Accel (in/s²)",    peakAccel);
        packet.put("Peak Decel (in/s²)",    peakDecel);
        packet.put("Tracking margin @ 94in", margin94);
        packet.put("Tracking margin @ 42in", margin42);
        dashboard.sendTelemetryPacket(packet);
    }
}
