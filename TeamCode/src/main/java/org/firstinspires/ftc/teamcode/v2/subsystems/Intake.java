package org.firstinspires.ftc.teamcode.v2.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
    InterpLUT transferPos    = new InterpLUT();
    private final MotorEx intake;
    private final DcMotorEx transfer;
    private final ServoEx latch;

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

    public Intake(final HardwareMap hMap, double distance) {

        this.distanceTo = distance;

        transferPos.add(0, 275);
        transferPos.add(29, 0.55);
        transferPos.add(33.5, 0.7);
        transferPos.add(43, 0.7);
        transferPos.add(49.5, 1);
        transferPos.add(56.5, 1);
        transferPos.add(68, 1);
        transferPos.add(76, 1);
        transferPos.add(93, 1);
        transferPos.add(1000, 1);
        transferPos.createLUT();


        intake = new MotorEx(hMap, "intake");
        transfer = hMap.get(DcMotorEx.class, "transfer");
        latch = new ServoEx(hMap, "latch");

        intake.setRunMode(MotorEx.RunMode.RawPower);
        intake.set(0);
        transfer.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        transfer.setPower(0);
        latch.set(LATCH_CLOSED);
    }

    // --- Collect mode commands (left trigger) ---

    public Command collectStart() {
        return new InstantCommand(() -> {
            shootMode = false;
            intake.set(INTAKE_SPEED);
            // Don't restart transfer if ball is already loaded at latch —
            // wait for shoot cycle (right trigger) to clear it first
            if (!ballLoaded) {
                startTransfer();
            }
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

    private void startTransfer() {
        transfer.setPower(transferPos.get(distanceTo));
        transferRunning = true;
        stallCount = 0;
        startupCount = 0;
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
        if (!transferRunning || shootMode) {
            return;
        }

        // Skip stall detection during motor startup ramp-up
        if (startupCount < STARTUP_IGNORE_LOOPS) {
            startupCount++;
            return;
        }

        double current = transfer.getCurrent(CurrentUnit.AMPS);
        Log.d("Transfer Current", String.valueOf(current));

        if (current > STALL_CURRENT) {
            stallCount++;
            Log.d("Transfer StallCount", String.valueOf(stallCount));
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
