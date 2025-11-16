package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTagID", group = "Vision")
public class AprilTagID extends OpMode {

    private Limelight3A limelight;

    // AprilTag IDs representing different patterns/motifs
    private static final int PATTERN_ID_A = 21;
    private static final int PATTERN_ID_B = 22;
    private static final int PATTERN_ID_C = 23;

    // Your AprilTag pipeline number
    private static final int LIMELIGHT_PIPELINE = 6;

    @Override
    public void init() {

        // Link Limelight via hardwareMap (MUST match config name!)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to your AprilTag pipeline
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        // Start polling data
        limelight.start();

        telemetry.addLine("Limelight initializing...");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine("No tag detected");
            telemetry.update();
            return;
        }

        // Read AprilTag results
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags.isEmpty()) {
            telemetry.addLine("No AprilTags found");
            telemetry.update();
            return;
        }

        // We only care about the FIRST detected tag
        LLResultTypes.FiducialResult tag = tags.get(0);
        int tagId = tag.getFiducialId();



        telemetry.addData("Detected Tag ID", tagId);
        telemetry.addData("X (deg)", "%.2f", tag.getTargetXDegrees());
        telemetry.addData("Y (deg)", "%.2f", tag.getTargetYDegrees());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
