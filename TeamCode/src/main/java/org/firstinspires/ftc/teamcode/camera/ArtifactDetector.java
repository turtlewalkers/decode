package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.json.JSONObject;

import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

@Disabled
@TeleOp(name = "ArtifactDetector", group = "Vision")
public class ArtifactDetector extends OpMode {

    private static final String LIMELIGHT_URL = "http://limelight.local:5801";
    private static final int LIMELIGHT_PIPELINE = 6;

    // AprilTag IDs representing different patterns/motifs
    private static final int PATTERN_ID_A = 21;
    private static final int PATTERN_ID_B = 22;
    private static final int PATTERN_ID_C = 23;

    @Override
    public void init() {
        setLimelightPipeline(LIMELIGHT_PIPELINE);
        telemetry.addLine("Limelight pipeline 6 ('AprilTags') selected");
        telemetry.update();
    }

    @Override
    public void loop() {
        try {
            JSONObject data = getLimelightData();
            if (data != null) {
                int tagId = data.optInt("tid", -1); // AprilTag ID from Limelight

                String pattern = "Unknown";
                if (tagId == PATTERN_ID_A) pattern = "Motif A";
                else if (tagId == PATTERN_ID_B) pattern = "Motif B";
                else if (tagId == PATTERN_ID_C) pattern = "Motif C";

                telemetry.addData("Detected Tag ID", tagId);
                telemetry.addData("Interpreted Pattern", pattern);
            } else {
                telemetry.addLine("No data from Limelight");
            }
        } catch (Exception e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
        telemetry.update();
    }

    private JSONObject getLimelightData() {
        try {
            URL url = new URL(LIMELIGHT_URL + "/json");
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");

            InputStream response = connection.getInputStream();
            Scanner scanner = new Scanner(response).useDelimiter("\\A");
            String json = scanner.hasNext() ? scanner.next() : "";
            scanner.close();

            return new JSONObject(json);
        } catch (Exception e) {
            return null;
        }
    }

    private void setLimelightPipeline(int pipeline) {
        try {
            URL url = new URL(LIMELIGHT_URL + "/setpipeline?p=" + pipeline);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.getInputStream().close();
        } catch (Exception ignored) {
        }
    }
}
