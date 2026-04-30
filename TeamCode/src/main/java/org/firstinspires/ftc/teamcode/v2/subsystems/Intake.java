package org.firstinspires.ftc.teamcode.v2.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.robot.Memory;

import java.util.function.Supplier;

@Config
public class Intake extends SubsystemBase {

    // --- Tunable via FTC Dashboard ---

    Follower follower;
    public static double STALL_CURRENT = 3.0;   // amps — between 0.25A normal and 9.2A stall
    public static int STALL_LOOPS = 2;           // consecutive loops above threshold → stop transfer (~40ms)
    public static int STARTUP_IGNORE_LOOPS = 10; // ignore stall for first N loops after transfer starts (~200ms)
    public static double INTAKE_SPEED = 1.0;
    public static double TRANSFER_SPEED = 1.0;
    public static double LATCH_OPEN = 0.71;
    public static double LATCH_CLOSED = 0.95;
    public static double SHOOT_SPEED = 1;
    public static int MAX_BALLS = 3;

    private final Supplier<Follower> followerSupplier;
    InterpLUT transferPos    = new InterpLUT();
    private final MotorEx intake;
    private final DcMotorEx transfer;
    private final ServoEx latch;
    private final DigitalChannel beamBreak2;
    private final DigitalChannel beamBreak3;

    private double shooterX, shooterY;

    // Transfer stall state
    private boolean transferRunning = false;
    private double distanceTo;
    private int stallCount = 0;
    private int startupCount = 0;

    // Ball loaded — set when transfer stalls in collect mode.
    // Prevents transfer from restarting on left trigger re-press until shoot clears it.
    private boolean ballLoaded = false;

    // Shoot mode flag — disables stall detection
    private boolean shootMode = false;

    // Ball counting via beam break
    private int ballCount = 0;
    private boolean prevBB2Broken = false;

    public Intake(final HardwareMap hMap,  Supplier<Follower> followerSupplier, double shooterX, double shooterY) {
        this.followerSupplier = followerSupplier;
        this.shooterX = shooterX;
        this.shooterY = shooterY;

        transferPos.add(0, 1);
        transferPos.add(43, 1);
        transferPos.add(59, 0.92);
        transferPos.add(67, 0.9);
        transferPos.add(75, 0.89);
        transferPos.add(83, 0.88);
        transferPos.add(91, 0.88);
        transferPos.add(99, 0.87);
        transferPos.add(107, 0.87);
        transferPos.add(118, 0.86);
        transferPos.add(131, 0.8);
        transferPos.add(147, 0.45);
        transferPos.add(3000, 0.45);
        transferPos.createLUT();


        intake = new MotorEx(hMap, "intake");
        transfer = hMap.get(DcMotorEx.class, "transfer");
        latch = new ServoEx(hMap, "latch");

        intake.setRunMode(MotorEx.RunMode.RawPower);
        intake.set(0);
        transfer.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        transfer.setPower(0);
        latch.set(LATCH_CLOSED);

        beamBreak2 = hMap.get(DigitalChannel.class, "beamBreak2");
        beamBreak3 = hMap.get(DigitalChannel.class, "beamBreak3");
        beamBreak2.setMode(DigitalChannel.Mode.INPUT);
        beamBreak3.setMode(DigitalChannel.Mode.INPUT);
    }

    // --- Collect mode commands (left trigger) ---

    public Command collectStart() {
        return new InstantCommand(() -> {
            shootMode = false;
            ballCount = 0;
            prevBB2Broken = false;
            intake.set(INTAKE_SPEED);
            // Don't restart transfer if ball is already loaded at latch —
            // wait for shoot cycle (right trigger) to clear it first
            if (!ballLoaded) {
                transfer.setPower(1);
                transferRunning = true;
                stallCount = 0;
                startupCount = 0;            }
        });
    }

    public Command collectStop() {
        return new InstantCommand(() -> {
            intake.set(0);
            stopTransfer();
            latch.set(LATCH_CLOSED);
        });
    }

    // --- Shoot mode commands (right trigger) ---

    public Command shootStart() {
        return new InstantCommand(() -> {
            SHOOT_SPEED = 0.75;
            shootMode = true;
            ballLoaded = false;  // clear loaded state — shoot cycle empties the latch
            stallCount = 0;
            latch.set(LATCH_OPEN);
            intake.set(INTAKE_SPEED);
            startTransfer();
        });
    }

    public Command shootStop() {
        return new InstantCommand(() -> {
            shootMode = false;
            SHOOT_SPEED = 1;
            latch.set(LATCH_CLOSED);
            intake.set(0);
            stopTransfer();
        });
    }

    // --- Reverse (dpad down) ---

    public Command reverse() {
        return new InstantCommand(() -> {
            shootMode = false;
            intake.set(-1.0);
            stopTransfer();
        });
    }

    // --- Internal helpers ---

    public int getBallCount() { return ballCount; }

    private void startTransfer() {
        distanceTo = distanceToTarget();
        transfer.setPower(transferPos.get(distanceTo));
        transferRunning = true;
        stallCount = 0;
        startupCount = 0;
    }
    private double distanceToTarget() {
        Pose robot = followerSupplier.get().getPose();
        double dx = shooterX - robot.getX();
        double dy = shooterY - robot.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    private void stopTransfer() {
        transfer.setPower(0);
        transferRunning = false;
        stallCount = 0;
        startupCount = 0;
    }

    // --- Periodic: stall detection (collect mode only) ---

    @Override
    public void periodic() {
        // Ball counting via BB2 — only after first ball is at latch, intake still running
        if (ballLoaded && !shootMode) {
            boolean bb2Broken = !beamBreak2.getState();
            if (bb2Broken && !prevBB2Broken) {
                ballCount++;
            }
            prevBB2Broken = bb2Broken;

            if (ballCount >= MAX_BALLS) {
                intake.set(0);
            }
        }

        if (!transferRunning || shootMode) {
            return;
        }

        distanceTo = distanceToTarget();
        transfer.setPower(transferPos.get(distanceTo));
        // Skip stall detection during motor startup ramp-up
        if (startupCount < STARTUP_IGNORE_LOOPS) {
            startupCount++;
            return;
        }

        double current = transfer.getCurrent(CurrentUnit.AMPS);
        if (Memory.debugMode) {
            Log.d("Transfer Current", String.valueOf(current));
            Log.d("Transfer Pos", String.valueOf(transferPos.get(distanceTo)));
            Log.d("Transfer Pos Dist", String.valueOf(distanceTo));
        }
        if (current > STALL_CURRENT) {
            stallCount++;
            if (Memory.debugMode) {
                Log.d("Transfer StallCount", String.valueOf(stallCount));
            }
            if (stallCount >= STALL_LOOPS) {
                Log.d("Transfer", "Stall detected — stopping transfer, ball loaded");
                ballLoaded = true;
                stopTransfer();
            }
        } else {
            stallCount = 0;
        }
    }
}
