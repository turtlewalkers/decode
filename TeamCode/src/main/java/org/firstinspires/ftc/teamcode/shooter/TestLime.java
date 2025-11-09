package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Timer;

@Config
@TeleOp
public class TestLime extends OpMode {
    public static Follower follower;
    private PIDController controller, controllerTurret;
    private TelemetryManager telemetryM;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.12, iT = 0, dT = 0;
    public static double f = 0.0265;
    private static double vel = 0;
    public static double target = 0;
    public static double alpha = 0.6;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    private DcMotorEx shooterb, shootert, intake, turret;
    private Servo hood;
    private VoltageSensor volt;
    public static double tangle = 40;
    public static double theta = 0;
    public static double shooterX = 135;
    public static double shooterY = 135;
    Servo latch;
    private double turretOffset = 0;
    private static final int TICKS_MIN = -330;
    private static final int TICKS_MAX = 990;
    private static final String LIMELIGHT_URL = "http://10.0.0.11:5801";
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
    public boolean stopAutoTurret = false;
    double horizontalAngle = 0;

    long cameraTimer = 0;
    @Override
    public void init() {
        if (Memory.allianceRed) {
            shooterY = 10;
        }
        controller = new PIDController(p, i, d);
        controllerTurret = new PIDController(pT, iT, dT);
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        RPM.add(0, 350);
        RPM.add(20, 350);
        RPM.add(39, 350);
        RPM.add(50, 375);
        RPM.add(60, 400);
        RPM.add(74, 415);
        RPM.add(90, 450);
        RPM.add(180, 450);
        RPM.createLUT();

        angle.add(0, 1);
        angle.add(20, 1);
        angle.add(39, 0.7);
        angle.add(50, 0.6);
        angle.add(60, 0.5);
        angle.add(74, 0.4);
        angle.add(90, 0.3);
        angle.add(180, 0.3);
        angle.createLUT();

        latch = hardwareMap.servo.get("latch");
    }

    @Override
    public void start() {
        follower = Constants.createFollower(hardwareMap);
        if (Memory.autoRan) {
            follower.setStartingPose(new Pose(50, 70, 0));
        } else {
            follower.setStartingPose(new Pose(72, 72, 90));
        }
        follower.startTeleOpDrive();
        follower.update();
        controller = new PIDController(p, i, d);
        Memory.autoRan = false;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            Memory.allianceRed = true;
        } else if (gamepad1.b) {
            Memory.allianceRed = false;
        }
        telemetry.addData("Alliance", Memory.allianceRed ? "Red" : "Blue");
    }

    @Override
    public void loop() {
        double multiplier = 1;
        if (gamepad1.left_trigger != 0) multiplier = 0.3;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * multiplier, -gamepad1.left_stick_x * multiplier, -gamepad1.right_stick_x * multiplier, true);
        follower.update();

        if (gamepad1.xWasPressed()) {
            stopAutoTurret = !stopAutoTurret;
        }

        latch.setPosition(gamepad1.y ? 1 : 0);
        if (cameraTimer == 10000) {
            horizontalAngle = getHorizontalAngle();
            cameraTimer = 0;
        }
        cameraTimer++;
        // Telemetry output
        telemetry.addData("Horizontal Angle (tx)", "%.2f°", horizontalAngle);

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();

        double dx = shooterX - robotX;
        double dy = shooterY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);
        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);
        telemetry.addData("autoX", Memory.robotAutoX);
        telemetry.addData("autoY", Memory.robotAutoY);
        telemetry.addData("autoHeading", Memory.robotHeading);

        telemetry.addData("Target Angle", targetAngleDeg);
        telemetry.addData("x", shooterX);
        telemetry.addData("y", shooterY);

        if (turretOffset <= 45 && turretOffset >= -45) {
            if (gamepad1.dpad_right && turretOffset > -45) {
                turretOffset -= 1;
            }
            if (gamepad1.dpad_left && turretOffset < 45) {
                turretOffset += 1;
            }
        }
        telemetry.addData("Target Angle", targetAngleDeg);
        targetAngleDeg += turretOffset;
        telemetry.addData("TurretOffset", turretOffset);
        targetAngleDeg = Math.max(targetAngleDeg, -30);
        targetAngleDeg = Math.min(targetAngleDeg, 240);
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        telemetry.addData("Turret Pos", turretPos);

//        targetTicks = Math.max(TICKS_MIN, Math.min(TICKS_MAX, targetTicks));
//        turretOffset += (gamepad2.right_trigger - gamepad2.left_trigger) * 5;
//        int offsetTicks = (int)(turretOffset * TICKS_PER_DEGREE);
//        int finalTargetTicks = targetTicks + offsetTicks;

        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        telemetry.addData("TurretPower", turretPower);
        telemetry.addData("stopAutoTurret", stopAutoTurret);

        if (!stopAutoTurret) {
            turret.setPower(turretPower);
        }
        intake.setPower(gamepad1.right_trigger);
        if (distance >0 && distance < 180) {
            target = RPM.get(distance);
            hood.setPosition(angle.get(distance));
        }
        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        vel = vel * alpha + shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - alpha);
        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        if (!gamepad1.a || robotX >= 40) {
            shooterb.setPower((pid + f * target) / presentVoltage);
            shootert.setPower((-1) * (pid + f * target) / presentVoltage);
        } else {
            shootert.setPower(0);
            shooterb.setPower(0);
        }
        if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }

//        if (gamepad1.dpad_left)
//        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Distance: ", distance);
        telemetry.addData("x: ", robotX);
        telemetry.addData("y: ", robotY);
        telemetry.addData("Heading", Math.toDegrees(robotHeading));
        telemetry.addData("RPM: ", RPM.get(distance));
        telemetry.addData("Angle: ", angle.get(distance));
        telemetry.update();
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