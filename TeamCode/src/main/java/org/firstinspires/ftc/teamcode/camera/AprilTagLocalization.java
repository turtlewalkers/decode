package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.TelemetryData;

import java.util.List;
@Config
@TeleOp
public class AprilTagLocalization extends OpMode {
    private FtcDashboard dashboard;
    private PIDController controller;
    private TelemetryManager telemetryM;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    private DcMotorEx turret;

    public static double TICKS_PER_DEG =
            ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    public static double TURRET_MIN = -90;
    public static double TURRET_MAX =  240;

    public static double kP = 0.03;
    public static double kI = 0.00000001;
    public static double kD = 0.00004;


    private PIDController turretPID;
    private Limelight3A limelight;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        turretPID = new PIDController(kP, kI, kD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();

        telemetry.addLine("waiting for start");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        double tx = 0;
        boolean hasTarget = false;


        if (result != null && result.isValid()) {
            tx = result.getTx();
            hasTarget = true;

        }

        int tagId = -1;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags != null && !tags.isEmpty()) {
            LLResultTypes.FiducialResult tag = tags.get(0);
            tagId = tag.getFiducialId();
        } else {
            hasTarget = false;
        }

        double turretPosDeg = turret.getCurrentPosition() / TICKS_PER_DEG;

        boolean goodtag = (tagId == 20 || tagId == 24);
        // just adding it to existing degree
        double turretTargetDeg = turretPosDeg - tx;
        //restrictions
        turretTargetDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretTargetDeg));

        turretPID.setPID(kP, kI, kD);

        double pidOut = turretPID.calculate(turretPosDeg, turretTargetDeg);

        if (hasTarget && goodtag) {
            turret.setPower(pidOut);
        } else {
            turret.setPower(0);
        }


        telemetry.addData("tx", tx);
        telemetry.addData("Has Tag", hasTarget);
        telemetry.addData("ID", tagId);
        telemetry.addData("Turret Pos (deg)", turretPosDeg);
        telemetry.addData("Target Deg", turretTargetDeg);
        telemetry.addData("PID Out", pidOut);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Position", turretPosDeg);
        packet.put("Turret Power", turret.getPower());
        packet.put("Target Position", turretTargetDeg);

        dashboard.sendTelemetryPacket(packet);

    }
}
