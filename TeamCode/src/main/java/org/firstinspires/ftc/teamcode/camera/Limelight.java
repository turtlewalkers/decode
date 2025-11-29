package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight tuning / inspection OpMode.
 *
 * - Shows status (fps, temp, pipeline).
 * - Filters fiducial results by tag ID and min area.
 * - Picks the "best" tag based on area (our stand-in for low ambiguity).
 * - Shows staleness + latencies so you can see how fresh the data is.
 *
 * Use this while you move the robot around by hand to learn where
 * tags are reliable and what quality numbers look like.
 */
@TeleOp(name = "LimelightTuning", group = "Test")
@Config
public class Limelight extends LinearOpMode {

    private static final String TAG = "LL_TUNING";

    // --- Tunable filters (FTC Dashboard) ---

    /** Keep only tags with this ID; -1 = accept any ID. */
    public static int TARGET_TAG_ID = -1;

    /** Minimum image area (0–100) for a tag to be considered "good". */
    public static double MIN_TAG_AREA = 0.5;

    /** Maximum allowed staleness in ms (how old the frame is). */
    public static long MAX_STALENESS_MS = 150;

    /** If true, log best-tag info to logcat/RobotLog periodically. */
    public static boolean LOG_TO_ROBOTLOG = true;

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: %s", e.getMessage());
            telemetry.update();
            return;
        }

        // Poll often – LL will internally cap at its max rate.
        limelight.setPollRateHz(100);

        // Use your AprilTag pipeline index here
        limelight.pipelineSwitch(6);

        limelight.start();

        telemetry.addLine("Limelight Tuning OpMode");
        telemetry.addLine("Press PLAY, then move the robot around by hand.");
        telemetry.update();

        waitForStart();

        long lastLogTime = 0;

        while (opModeIsActive()) {

            // --- Status block (health, pipeline, etc.) ---
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", status.getName());
            telemetry.addData("LL Temp (C)", "%.1f", status.getTemp());
            telemetry.addData("LL CPU (%)", "%.1f", status.getCpu());
            telemetry.addData("LL FPS", (int) status.getFps());
            telemetry.addData("Pipeline", "idx=%d type=%s",
                    status.getPipelineIndex(), status.getPipelineType().toString());

            // --- Get latest result ---
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid LLResult");
                telemetry.update();
                continue;
            }

            long staleness = result.getStaleness(); // ms since capture
            double capLat   = result.getCaptureLatency();   // ms
            double tgtLat   = result.getTargetingLatency(); // ms
            double parseLat = result.getParseLatency();     // ms

            telemetry.addData("Staleness (ms)", staleness);
            telemetry.addData("Cap+Tgt Lat (ms)", "%.1f", capLat + tgtLat);
            telemetry.addData("Parse Lat (ms)", "%.1f", parseLat);

            // --- Base tx/ty/ta (whole pipeline) ---
            telemetry.addData("tx", "%.2f", result.getTx());
            telemetry.addData("ty", "%.2f", result.getTy());
            telemetry.addData("ta", "%.2f", result.getTa());

            // --- Fiducial list (per-tag results) ---
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            telemetry.addData("#Fiducials", tags.size());

            LLResultTypes.FiducialResult best = null;
            double bestArea = 0.0;

            if (staleness <= MAX_STALENESS_MS) {
                for (LLResultTypes.FiducialResult fr : tags) {

                    int id = fr.getFiducialId();
                    double area = fr.getTargetArea(); // 0–100 image %

                    // Optional tag-ID filter
                    if (TARGET_TAG_ID >= 0 && id != TARGET_TAG_ID) {
                        continue;
                    }

                    // Area filter (stand-in for "low ambiguity")
                    if (area < MIN_TAG_AREA) {
                        continue;
                    }

                    if (best == null || area > bestArea) {
                        best = fr;
                        bestArea = area;
                    }
                }
            }

            if (best != null) {
                int id = best.getFiducialId();
                double tx = best.getTargetXDegrees();
                double ty = best.getTargetYDegrees();
                double area = best.getTargetArea();

                Pose3D poseField = best.getRobotPoseFieldSpace();

                telemetry.addLine("== Best Tag ==");
                telemetry.addData("ID", id);
                telemetry.addData("Area", "%.2f", area);
                telemetry.addData("tx_deg", "%.2f", tx);
                telemetry.addData("ty_deg", "%.2f", ty);
                if (poseField != null) {
                    telemetry.addData("FieldPose",
                            "(%.1f, %.1f, %.1f) yaw=%.1f",
                            poseField.getPosition().x,
                            poseField.getPosition().y,
                            poseField.getPosition().z,
                            poseField.getOrientation().getYaw(AngleUnit.DEGREES));
                }

                // Optional logging so you can copy values from logcat
                long now = System.currentTimeMillis();
                if (LOG_TO_ROBOTLOG && now - lastLogTime > 500) {
                    RobotLog.ii(TAG,
                            "BEST id=%d area=%.2f tx=%.2f ty=%.2f stale=%dms",
                            id, area, tx, ty, staleness);
                    if (poseField != null) {
                        RobotLog.ii(TAG,
                                "POSE field: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                                poseField.getPosition().x,
                                poseField.getPosition().y,
                                poseField.getPosition().z,
                                poseField.getOrientation().getYaw(AngleUnit.DEGREES));
                    }
                    lastLogTime = now;
                }
            } else {
                if (staleness > MAX_STALENESS_MS) {
                    telemetry.addLine("All tags rejected: too stale");
                } else if (tags.isEmpty()) {
                    telemetry.addLine("No fiducials in frame");
                } else {
                    telemetry.addLine("All tags rejected by ID/area filters");
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
