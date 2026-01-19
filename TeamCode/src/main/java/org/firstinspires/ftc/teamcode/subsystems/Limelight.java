package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterMove.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterMove.TURRET_MIN;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterMove.turretOffset;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Memory;

import java.util.List;
import java.util.function.Supplier;
@Config
public class Limelight extends SubsystemBase {
    public static double kP = 0.032;
    public static double kI = 0.0000001;
    public static double kD = 0.00004;
    private PIDController turretPID;
    private Limelight3A limelight;
    private static final double METERS_TO_INCHES = 39.37;
    private final Supplier<Follower> followerSupplier;
    public static boolean fix = false;
    public static boolean turretOn = false;
    public static double power = 0;

    private double frozenTx = 0;
    private boolean txFrozen = false;



    public static double TICKS_PER_DEG =
            ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;




    public Limelight(final HardwareMap hMap, Supplier<Follower> followerSupplier) {
        this.followerSupplier = followerSupplier;
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();
        turretPID = new PIDController(kP, kI, kD);
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    public Command relocalize() {
        return new InstantCommand(() -> fix = true);
    }

    public Command norelocalize() {
        return new InstantCommand(() -> fix = false);
    }

    public Command fixTurret() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> turretOn = true)
        );
    }

    public Command nofixTurret() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> fix = false),
                new InstantCommand(() -> turretOn = false)
        );
    }

    @Override
    public void periodic() {
        boolean hasTarget = false;
        if (fix || turretOn) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                hasTarget = true;

                Pose3D botpose = result.getBotpose();

                double llX_m = botpose.getPosition().x;
                double llY_m = botpose.getPosition().y;

                double llX_in = llX_m * METERS_TO_INCHES;
                double llY_in = llY_m * METERS_TO_INCHES;

                double decodeX = 72 + llY_in;
                double decodeY = 72 - llX_in;

                if (fix) {
                    Follower follower = followerSupplier.get();
                    double robotHeading = followerSupplier.get().getHeading();
                    follower.setPose(new Pose(decodeX, decodeY, robotHeading));
                }

                if (turretOn) {
                    int targetId = 20;
                    if (Memory.allianceRed) targetId = 24;
                    int tagId = -1;

                    double tx = result.getTx();

                    if (!txFrozen) {
                        frozenTx = tx;
                        txFrozen = true;
                        Log.d("frozenTx", String.valueOf(frozenTx));
                    }


                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null && !tags.isEmpty()) {
                        LLResultTypes.FiducialResult tag = tags.get(0);
                        tagId = tag.getFiducialId();
                    } else {
                        hasTarget = false;
                    }

                    boolean goodtag = (tagId == targetId);

                    double turretPosDeg = ShooterMove.turretPos;

                    double turretTargetDeg = turretPosDeg - tx;
                    turretTargetDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretTargetDeg));

                    turretPID.setPID(kP, kI, kD);

                    double pidOut = turretPID.calculate(turretPosDeg, turretTargetDeg);



                    if (turretOn && hasTarget && goodtag) {
                        power = pidOut;
                        if (turretTargetDeg - turretPosDeg <= 3) {
                            frozenTx = turretOffset;
                            Log.d("turretOffset",String.valueOf(turretOffset));
                        }
                        else {
                            power = 0;
                            txFrozen = false;
                        }


                    }
                }
            }
        }
    }
}