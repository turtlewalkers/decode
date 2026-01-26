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
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
import org.firstinspires.ftc.teamcode.subsystems.ShooterMove;

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
    private VoltageSensor volt;


    public double frozenTx = 0;
    private boolean txFrozen = false;
    private ShooterMove currShooter;

    public static boolean hasValidTarget = false;



    public static double TICKS_PER_DEG =
            ((((1.0 + (46.0 / 17.0))) * (1.0 + (46.0 / 11.0))) * 28.0 * 3.0) / 360.0;


    public Limelight(final HardwareMap hMap, Supplier<Follower> followerSupplier, ShooterMove shooter) {
        this.followerSupplier = followerSupplier;
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();
        currShooter = shooter;
        volt = hMap.get(VoltageSensor.class, "Control Hub");

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
        return new InstantCommand(() -> turretOn = true);
    }

    public Command nofixTurret() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> fix = false),
                new InstantCommand(() -> turretOn = false)
        );
    }

    @Override
    public void periodic() {
        Log.d("RB", "turretOn=" + turretOn);

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
                    int targetId = Memory.allianceRed ? 24 : 20;
                    int tagId = -1;

                    // Live tx: ALWAYS used for PID + targeting
                    double tx = result.getTx();

                    Log.d("tx", String.valueOf(tx));

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null && !tags.isEmpty()) {
                        tagId = tags.get(0).getFiducialId();
                    } else {
                        hasTarget = false;
                    }

                    boolean goodtag = (tagId == targetId);

                    hasValidTarget = hasTarget && goodtag;


                    double turretPosDeg = ShooterMove.turretPos;
                    Log.d("turretPosDeg", String.valueOf(turretPosDeg));

                    double turretTargetDeg = turretPosDeg - tx;
                    turretTargetDeg = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretTargetDeg));
                    Log.d("turretTargetDeg", String.valueOf(turretTargetDeg));

                    double pidOut = currShooter.controllerTurret.calculate(turretPosDeg, turretTargetDeg);
                    //double pidOut = currShooter.controllerTurret.calculate(0, 15);
                    Log.d("PIDPower", String.valueOf(pidOut));

                    if (!txFrozen) {
                        frozenTx = tx;
                        txFrozen = true;
                    }
                    Log.d("prefrozentx", String.valueOf( frozenTx));
                    double presentVoltage = volt.getVoltage();
                    Log.d("presentVoltage", String.valueOf( presentVoltage));
                    if (hasTarget && goodtag) {
                        power = pidOut;
                        Log.d("PIDPower", String.valueOf(power));
                        //currShooter.turret.set(power/presentVoltage);
                        //currShooter.turret.set(power);
                        double fixedtx = result.getTx();
                        Log.d("fixedtx", String.valueOf(fixedtx));
                        Log.d("fixedturretPosDeg", String.valueOf(ShooterMove.turretPos));

                        if (Math.abs(fixedtx) <= 4.0) {
                            currShooter.SetTurretOffset(frozenTx+fixedtx);
                            Log.d("frozenTx", "frozentxoffset=" + frozenTx);
                            Log.d("turretOffset", "turretOffset=" + turretOffset);
                        }

                    } else {
                        power = 0;
                    }
                }
            }
        }

        // Reset freeze ONLY when turret is explicitly turned off
        if (!turretOn) {
            txFrozen = false;
        }
    }
}
