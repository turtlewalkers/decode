package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.List;

public class LimeMoving extends SubsystemBase {

    public static final double TICKS_PER_DEG =
            ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0 * 3.0) / 360.0;

    public static final double TURRET_MIN = -90;
    public static final double TURRET_MAX = 240;

    public static double kP = 0.03;
    public static double kI = 0.00000001;
    public static double kD = 0.00004;

    private final MotorEx turret;
    private final Limelight3A limelight;
    private final PIDController controllerTurret;

    public static boolean turretOn = false;
    public static double power = 0;
    public static boolean debugMode = false;

    public LimeMoving(HardwareMap hMap) {
        turret = new MotorEx(hMap, "turret");
        turret.stopAndResetEncoder();
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setRunMode(MotorEx.RunMode.RawPower);

        controllerTurret = new PIDController(kP, kI, kD);

        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();
    }

    @Override
    public void periodic() {
        controllerTurret.setPID(kP, kI, kD);

        LLResult result = limelight.getLatestResult();
        boolean hasTarget = false;
        boolean goodTag = false;
        double tx = 0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null && !tags.isEmpty()) {
                LLResultTypes.FiducialResult tag = tags.get(0);
                int tagId = tag.getFiducialId();
                goodTag = (tagId == 20 || tagId == 24);
                hasTarget = goodTag;
                tx = result.getTx();
            }
        }

        double turretPosDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        double turretTargetDeg = turretPosDeg - tx;
        turretTargetDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretTargetDeg));

        double turretPID = controllerTurret.calculate(turretPosDeg, turretTargetDeg);

        if (turretOn && (hasTarget || debugMode)) {
            turret.set(turretPID);
            power = turretPID;
        } else {
            turret.set(0);
            power = 0;
        }

        Log.d("LimeMoving", String.format(
                "turretOn=%b, debugMode=%b, hasTarget=%b, goodTag=%b, tx=%.2f, turretPosDeg=%.2f, turretPower=%.2f",
                turretOn, debugMode, hasTarget, goodTag, tx, turretPosDeg, power
        ));
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
