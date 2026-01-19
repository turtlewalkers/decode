package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp(name = "AprilTagDistance", group = "Camera")
public class AprilTagDistance extends LinearOpMode {

    // Limelight configuration
    private static final String LIMELIGHT_URL = "http://10.0.0.11:5801";

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            double horizontalAngle = getHorizontalAngle();

            // Telemetry output
            telemetry.addData("Horizontal Angle (tx)", "%.2f°", horizontalAngle);
            telemetry.update();

        }
    }

    private double getHorizontalAngle() {
        try {
            URL url = new URL(LIMELIGHT_URL + "/json");
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(200);
            connection.setReadTimeout(200);

            BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            StringBuilder response = new StringBuilder();
            String line;
            while ((line = in.readLine()) != null) response.append(line);
            in.close();

            JSONObject json = new JSONObject(response.toString());
            JSONArray fiducials = json.getJSONObject("Results").optJSONArray("Targets_Fiducials");

            if (fiducials != null && fiducials.length() > 0) {
                JSONObject tag = fiducials.getJSONObject(0);
                return tag.optDouble("txnc", 0.0); // horizontal offset in degrees
            }

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }

        return 0.0;
    }
}
