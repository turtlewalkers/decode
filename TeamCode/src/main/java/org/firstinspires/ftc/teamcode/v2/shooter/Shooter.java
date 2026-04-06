package org.firstinspires.ftc.teamcode.v2.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Flywheel PID tuning opmode.
 * Both vel and target are in RPM — tuned p/i/d values copy directly to ShooterMove.
 *
 * === SEQUENCE ===
 * Left trigger (hold)  — collect: intake + transfer run, first ball stalls transfer
 *                        → transfer stops, all 3 balls held at closed latch
 * Right trigger (hold) — shoot: latch open + transfer run → all 3 balls fire
 * Release right        — latch close, transfer stop
 * DPAD up/down         — adjust TARGET_RPM by 10
 *
 * === PID TUNING PROCEDURE ===
 * 1. Set ENABLE_FF = false, TARGET_RAD = 300 (rad/s ≈ 2865 RPM)
 * 2. Increase p until velocity reaches target without oscillation
 * 3. Add i to eliminate steady-state error (keep small to avoid windup)
 * 4. Set ENABLE_FF = true with kV/kS from ShooterTest for best performance
 * 5. Load 3 balls (left trigger), then shoot (right trigger)
 *    Watch RPM dip per ball on Dashboard — tune until recovery is fast between balls
 * 6. Copy confirmed p/i/d to ShooterMove
 */
@Config
@TeleOp(name = "Shooter", group = "V2")
public class Shooter extends OpMode {

    // --- Flywheel PID + FF ---
    public static double p = 0.4, i = 0.05, d = 0;
    public static double f = 0.0025;
    public static double TARGET_RAD = 300;        // rad/s — 300 rad/s ≈ 2865 RPM (~0.55 power)
    public static boolean ENABLE_FF = true;
    public static double kV = 0.021477551;
    public static double kS = 0.760983135;

    // --- Hood ---
    public static boolean HOOD_ENABLED = true;
    public static double theta = 0;          // hood position [0, 1]

    // --- Intake / transfer / latch ---
    public static boolean INTAKE_ENABLED     = true;
    public static double  INTAKE_SPEED       = 1.0;
    public static double  TRANSFER_SPEED     = 1.0;
    public static double  LATCH_OPEN         = 0.71;
    public static double  LATCH_CLOSED       = 0.95;

    // --- Stall detection (collect mode) ---
    public static double STALL_CURRENT        = 3.0;  // amps
    public static int    STALL_LOOPS          = 2;    // consecutive loops above threshold → stall confirmed
    public static int    STARTUP_IGNORE_LOOPS = 10;   // ignore stall for first N loops after transfer starts

    // Hardware
    private FtcDashboard dashboard;
    private PIDController controller;
    private DcMotorEx shooterB, shooterT;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo hood, latch;
    private VoltageSensor volt;

    // Collect state
    private boolean transferStalled = false;
    private int stallCount    = 0;
    private int startupCount  = 0;

    private boolean lastLeft  = false;
    private boolean lastRight = false;
    private boolean lastUp    = false;
    private boolean lastDown  = false;

    @Override
    public void init() {
        dashboard  = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);

        shooterB = hardwareMap.get(DcMotorEx.class, "sb");
        shooterT = hardwareMap.get(DcMotorEx.class, "st");
        shooterB.setDirection(DcMotorSimple.Direction.REVERSE);
        volt     = hardwareMap.get(VoltageSensor.class, "Control Hub");

        if (INTAKE_ENABLED) {
            intakeMotor   = hardwareMap.get(DcMotorEx.class, "intake");
            transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
            latch         = hardwareMap.get(Servo.class, "latch");
            transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            latch.setPosition(LATCH_CLOSED);
        }

        hood = HOOD_ENABLED ? hardwareMap.get(Servo.class, "hood") : null;

        telemetry.addData("Status", "Ready — LEFT=collect  RIGHT=shoot");
        telemetry.addData("TARGET_RAD",     TARGET_RAD);
        telemetry.addData("INTAKE_ENABLED", INTAKE_ENABLED);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean leftTrigger  = gamepad1.left_trigger  > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean up           = gamepad1.dpad_up;
        boolean down         = gamepad1.dpad_down;

        // DPAD — adjust TARGET_RAD live
        if (up   && !lastUp)   TARGET_RAD += 10;
        if (down && !lastDown) TARGET_RAD -= 10;
        lastUp = up; lastDown = down;

        // Hood
        if (HOOD_ENABLED && hood != null) hood.setPosition(theta);

        // --- Intake / transfer / latch logic ---
        if (INTAKE_ENABLED) {
            if (rightTrigger) {
                // Shoot — latch open, transfer run continuously (no stall detection)
                latch.setPosition(LATCH_OPEN);
                intakeMotor.setPower(INTAKE_SPEED);
                transferMotor.setPower(TRANSFER_SPEED);
                // Reset collect state so next left trigger works fresh
                transferStalled = false;
                stallCount      = 0;
                startupCount    = 0;

            } else if (leftTrigger) {
                // Collect — intake + transfer run, stall stops transfer (first ball blocks latch)
                latch.setPosition(LATCH_CLOSED);
                intakeMotor.setPower(INTAKE_SPEED);

                if (!transferStalled) {
                    transferMotor.setPower(TRANSFER_SPEED);

                    // Stall detection
                    if (startupCount < STARTUP_IGNORE_LOOPS) {
                        startupCount++;
                    } else {
                        double current = transferMotor.getCurrent(CurrentUnit.AMPS);
                        if (current > STALL_CURRENT) {
                            stallCount++;
                            if (stallCount >= STALL_LOOPS) {
                                transferStalled = true;
                                transferMotor.setPower(0);
                            }
                        } else {
                            stallCount = 0;
                        }
                    }
                }
                // If already stalled, intake keeps running to push remaining balls up to latch

            } else {
                // Neither trigger — stop everything, close latch
                latch.setPosition(LATCH_CLOSED);
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                // Reset stall state when left trigger released so next press works fresh
                if (lastLeft) {
                    transferStalled = false;
                    stallCount      = 0;
                    startupCount    = 0;
                }
            }
        }

        lastLeft  = leftTrigger;
        lastRight = rightTrigger;

        // --- Flywheel PID + FF (always running) ---
        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        double vel = shooterB.getVelocity() * (2 * Math.PI / 28);  // ticks/sec → rad/s (matches V1)

        double pid = controller.calculate(vel, TARGET_RAD);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));

        double ffVolts = ENABLE_FF ? (kV * TARGET_RAD + kS * Math.signum(TARGET_RAD)) : f * TARGET_RAD;
        double flywheelVolts = Math.max(-presentVoltage, Math.min(pid + ffVolts, presentVoltage));

        shooterB.setPower(flywheelVolts / presentVoltage);
        shooterT.setPower(flywheelVolts / presentVoltage);

        // --- Telemetry ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity (rad/s)", vel);
        packet.put("Target (rad/s)",   TARGET_RAD);
        packet.put("Error (rad/s)",    TARGET_RAD - vel);
        packet.put("PID output",       pid);
        packet.put("FF volts",         ffVolts);
        packet.put("Battery V",        presentVoltage);
        packet.put("transferStalled",  transferStalled);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("transferStalled",  transferStalled);
        telemetry.addData("Velocity (rad/s)", "%.1f", vel);
        telemetry.addData("Target (rad/s)",   "%.1f", TARGET_RAD);
        telemetry.addData("Error (rad/s)",    "%.1f", TARGET_RAD - vel);
        telemetry.addData("Battery V",        "%.2f", presentVoltage);
        telemetry.addData("DPAD up/down",     "adjust TARGET_RAD (now %.0f)", TARGET_RAD);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterB.setPower(0);
        shooterT.setPower(0);
        if (INTAKE_ENABLED) {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
            latch.setPosition(LATCH_CLOSED);
        }
    }
}
