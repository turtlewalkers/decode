package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.Memory;

import java.util.function.Supplier;

public class Limelight extends SubsystemBase {
    private Limelight3A limelight;
    private static final double METERS_TO_INCHES = 39.37;
    private final Supplier<Follower> followerSupplier;
    private boolean fix = false, turret = false;

    public Limelight(final HardwareMap hMap, Supplier<Follower> followerSupplier) {
        this.followerSupplier = followerSupplier;
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6);
        limelight.start();
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
                new InstantCommand(() -> fix = true),
                new InstantCommand(() -> turret = true)
        );
    }

    public Command nofixTurret() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> fix = false),
                new InstantCommand(() -> turret = false)
        );
    }

    @Override
    public void periodic() {
        if (fix) {
            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {

                Pose3D botpose = result.getBotpose();

                double llX_m = botpose.getPosition().x;
                double llY_m = botpose.getPosition().y;

                double llX_in = llX_m * METERS_TO_INCHES;
                double llY_in = llY_m * METERS_TO_INCHES;

                double decodeX = 72 + llY_in;
                double decodeY = 72 - llX_in;

                Follower follower = followerSupplier.get();
                double robotHeading = followerSupplier.get().getHeading();
                follower.setPose(new Pose(decodeX, decodeY, robotHeading));

                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    int targetId = 20;
                    if (Memory.allianceRed) targetId = 24;
                    if (targetId == fr.getFiducialId() && turret) {
                        double angle = fr.getTargetXDegrees();
                        Log.d("tX", String.valueOf(fr.getTargetXDegrees()));
                        ShooterMove.turretOffset = angle;
                    }
                }
            }
        }
    }
}