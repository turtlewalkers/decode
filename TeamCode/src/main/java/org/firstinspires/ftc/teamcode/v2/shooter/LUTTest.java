package org.firstinspires.ftc.teamcode.v2.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.controller.PIDController;

import java.util.List;

/**
 * LUT tuning opmode.
 *
 * Place robot at a measured distance along the diagonal from the goal.
 * Set TEST_DISTANCE, TARGET_RPM, TARGET_HOOD on Dashboard to match.
 * Load balls with left trigger, shoot with right trigger, observe scoring.
 * Adjust TARGET_RPM / TARGET_HOOD until consistent. Record values.
 * Move robot to next distance and repeat.
 *
 * === WORKFLOW ===
 * 1. Move robot along field diagonal to desired distance (tape measure)
 * 2. Set TEST_DISTANCE on Dashboard (informational — not used in control)
 * 3. Set TARGET_RPM (rad/s) — start from V1 value for that distance
 * 4. Set TARGET_HOOD [0.0–1.0] — start from V1 value (HOOD_ENABLED=false skips hood)
 * 5. Left trigger — load 3 balls
 * 6. Right trigger — shoot all balls
 * 7. Watch FlywheelRPM on Dashboard — should reach TARGET_RPM before ball arrives
 * 8. Adjust TARGET_RPM up/down until balls score consistently
 * 9. Copy confirmed values into ShooterMove LUT at that distance breakpoint
 *
 * === STARTING VALUES (V1, unverified on V2) ===
 *   42"  → rpm=280, hood=0.70
 *   70"  → rpm=340, hood=0.21
 *   91"  → rpm=370, hood=0.15
 *  114"  → rpm=415, hood=0.10
 *  131"  → rpm=445, hood=0.03
 *
 * === CONTROLS ===
 * Left trigger  — collect (intake + transfer, stall stops transfer at latch)
 * Right trigger — shoot (latch open, transfer runs)
 * Release       — stop everything
 * DPAD UP/DOWN  — TARGET_RPM ±10 rad/s
 * A             — flywheel on
 * B             — flywheel off
 */
@Config
@TeleOp(name = "LUTTest", group = "V2")
public class LUTTest extends OpMode {

    // --- Set these per shot on Dashboard ---
    public static double TEST_DISTANCE = 42.0;   // inches — informational label only
    public static double TARGET_RPM    = 280.0;  // rad/s target for flywheel
    public static double TARGET_HOOD   = 0.50;   // [0,1] servo position (ignored if HOOD_ENABLED=false)
    public static boolean HOOD_ENABLED = true;

    // --- PID + FF (copy from ShooterMove — retune on V2) ---
    public static double p = 65, i = 0.02, d = 0.0035;
    public static double f = 0.0025;
    public static boolean ENABLE_FF = false;
    public static double kV = 0.02049082;
    public static double kS = 0.499555731;

    // --- Intake / transfer / latch ---
    public static double INTAKE_SPEED   = 1.0;
    public static double TRANSFER_SPEED = 1.0;
    public static double LATCH_OPEN     = 0.71;
    public static double LATCH_CLOSED   = 0.95;

    // --- Stall detection (collect mode) ---
    public static double STALL_CURRENT        = 3.0;
    public static int    STALL_LOOPS          = 2;
    public static int    STARTUP_IGNORE_LOOPS = 10;

    private FtcDashboard dashboard;
    private PIDController controller;
    private DcMotorEx shooterB, shooterT;
    private DcMotorEx intakeMotor, transferMotor;
    private Servo hood, latch;
    private VoltageSensor volt;
    private List<LynxModule> allHubs;

    private boolean flywheelOn = true;

    // Stall state
    private boolean transferStalled = false;
    private int stallCount   = 0;
    private int startupCount = 0;

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

        intakeMotor   = hardwareMap.get(DcMotorEx.class, "intake");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        latch         = hardwareMap.get(Servo.class, "latch");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setPosition(LATCH_CLOSED);

        //hood = HOOD_ENABLED ? hardwareMap.get(Servo.class, "hood") : null;
        hood = hardwareMap.get(Servo.class, "hood");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        hood.setDirection(Servo.Direction.REVERSE);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addData("Status", "Ready — left=collect  right=shoot");
        telemetry.addData("TEST_DISTANCE", TEST_DISTANCE);
        telemetry.addData("TARGET_RPM",    TARGET_RPM);
        telemetry.update();


        /*

        distance, rad, hood, transferspeed
        29, 275, 1, 0.55
        35.5, 275, 0.7, 0.7
        43, 275, 0.7, 1
        49.5, 285, 0.65, 1
        56.5, 290. 0.58, 1
        68, 310, 0.5, 1
        76, 315, 0.45, 1
        83, 335, 0.40, 1
        91, 360, 0.30, 1
         */
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        boolean leftTrigger  = gamepad1.left_trigger  > 0.5;
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean up           = gamepad1.dpad_up;
        boolean down         = gamepad1.dpad_down;

        // DPAD — adjust TARGET_RPM live
        if (up   && !lastUp)   TARGET_RPM += 10;
        if (down && !lastDown) TARGET_RPM -= 10;
        lastUp = up; lastDown = down;

        // A/B — flywheel on/off
        if (gamepad1.a) flywheelOn = true;
        if (gamepad1.b) flywheelOn = false;

        // Hood
        if (HOOD_ENABLED && hood != null) hood.setPosition(TARGET_HOOD);

        // --- Intake / transfer / latch ---
        if (rightTrigger) {
            latch.setPosition(LATCH_OPEN);
            intakeMotor.setPower(INTAKE_SPEED);
            transferMotor.setPower(TRANSFER_SPEED);
            transferStalled = false;
            stallCount      = 0;
            startupCount    = 0;

        } else if (leftTrigger) {
            latch.setPosition(LATCH_CLOSED);
            intakeMotor.setPower(INTAKE_SPEED);

            if (!transferStalled) {
                transferMotor.setPower(TRANSFER_SPEED);
                if (startupCount < STARTUP_IGNORE_LOOPS) {
                    startupCount++;
                } else {
                    double current = transferMotor.getCurrent(
                            org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
                    if (current > STALL_CURRENT) {
                        if (++stallCount >= STALL_LOOPS) {
                            transferStalled = true;
                            transferMotor.setPower(0);
                        }
                    } else {
                        stallCount = 0;
                    }
                }
            }

        } else {
            latch.setPosition(LATCH_CLOSED);
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
            if (lastLeft) {
                transferStalled = false;
                stallCount      = 0;
                startupCount    = 0;
            }
        }

        lastLeft  = leftTrigger;
        lastRight = rightTrigger;

        // --- Flywheel PID + FF ---
        double presentVoltage = volt.getVoltage();
        double vel = shooterB.getVelocity() * (2 * Math.PI / 28);  // rad/s

        if (flywheelOn) {
            controller.setPID(p, i, d);
            double pid = controller.calculate(vel, TARGET_RPM);
            pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));

            double ffVolts = ENABLE_FF
                    ? (kV * TARGET_RPM + kS * Math.signum(TARGET_RPM))
                    : f * TARGET_RPM;
            double flywheelVolts = Math.max(-presentVoltage, Math.min(pid + ffVolts, presentVoltage));

            shooterB.setPower(flywheelVolts / presentVoltage);
            shooterT.setPower(flywheelVolts / presentVoltage);
        } else {
            shooterB.setPower(0);
            shooterT.setPower(0);
        }

        // --- Telemetry ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity (rad/s)", vel);
        packet.put("Target (rad/s)",   TARGET_RPM);
        packet.put("Error (rad/s)",    TARGET_RPM - vel);
        packet.put("Battery V",        presentVoltage);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Distance (in)",     TEST_DISTANCE);
        telemetry.addData("Target (rad/s)",    "%.1f", TARGET_RPM);
        telemetry.addData("Velocity (rad/s)",  "%.1f", vel);
        telemetry.addData("Error (rad/s)",     "%.1f", TARGET_RPM - vel);
        telemetry.addData("Hood position",     TARGET_HOOD);
        telemetry.addData("transferStalled",   transferStalled);
        telemetry.addData("Battery V",         "%.2f", presentVoltage);
        telemetry.addData("DPAD up/down",      "TARGET_RPM ±10  (now %.0f)", TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterB.setPower(0);
        shooterT.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        latch.setPosition(LATCH_CLOSED);
    }
}
