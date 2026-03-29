package org.firstinspires.ftc.teamcode.v2.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

/**
 * Hardware health check opmode.
 * Tests each motor and servo individually at low power.
 * Expand with more hardware as phases are completed.
 *
 * Phase 1: intake, transfer, latch
 * Phase 3: tl1, tl2, hood (add when ready)
 * Phase 4: sb, st flywheel (add when ready)
 */
@Config
@TeleOp(name = "HardwareTest", group = "V2")
public class HardwareTest extends OpMode {

    public static double MOTOR_TEST_POWER = 0.3;  // slow power for safety
    public static double LATCH_OPEN = 0.71;
    public static double LATCH_CLOSED = 0.95;
    public static double STALL_CURRENT = 3.0;     // amps — stop transfer if exceeded
    public static int STALL_LOOPS = 2;             // consecutive loops above threshold → stop

    // Phase 1
    private MotorEx intake;
    private DcMotorEx transfer;
    private ServoEx latch;

    // Phase 3 — uncomment when turret is ready
    // private ServoEx turretL1, turretL2, hood;

    // Phase 4 — uncomment when flywheel is ready
    // private MotorEx shooterB, shooterT;

    private double latchPosition = LATCH_CLOSED;
    private int stallCount = 0;
    private boolean transferStalled = false;
    private int loopCount = 0;

    @Override
    public void init() {
        // Phase 1
        intake = new MotorEx(hardwareMap, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        latch = new ServoEx(hardwareMap, "latch");

        intake.setRunMode(MotorEx.RunMode.RawPower);
        intake.set(0);
        transfer.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        transfer.setPower(0);
        latch.set(LATCH_CLOSED);

        // Phase 3 — uncomment when turret is ready
        // turretL1 = new ServoEx(hardwareMap, "tl1");
        // turretL2 = new ServoEx(hardwareMap, "tl2");
        // hood = new ServoEx(hardwareMap, "hood");

        // Phase 4 — uncomment when flywheel is ready
        // shooterB = new MotorEx(hardwareMap, "sb");
        // shooterT = new MotorEx(hardwareMap, "st");

        telemetry.addData("Status", "Initialized — ready to test");
        telemetry.addData("Controls", "");
        telemetry.addData("  LB (hold)", "Intake forward");
        telemetry.addData("  RB (hold)", "Transfer forward");
        telemetry.addData("  A  (hold)", "Intake reverse");
        telemetry.addData("  B  (hold)", "Transfer reverse");
        telemetry.addData("  DPAD UP", "Latch open (1.0)");
        telemetry.addData("  DPAD DOWN", "Latch closed (0.0)");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopCount++;

        // --- Intake motor ---
        double intakePower = 0;
        if (gamepad1.left_bumper) {
            intakePower = MOTOR_TEST_POWER;
        } else if (gamepad1.a) {
            intakePower = -MOTOR_TEST_POWER;
        }
        intake.set(intakePower);

        // --- Transfer motor ---
        double transferPower = 0;
        if (gamepad1.right_bumper) {
            transferPower = MOTOR_TEST_POWER;
        } else if (gamepad1.b) {
            transferPower = -MOTOR_TEST_POWER;
        }

        // Stall detection — only when running forward (RB), latch may be closed
        double transferCurrent = transfer.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        if (transferPower > 0) {
            if (transferCurrent > STALL_CURRENT) {
                stallCount++;
                if (stallCount >= STALL_LOOPS) {
                    transferStalled = true;
                }
            } else {
                stallCount = 0;
            }
        } else {
            // Reset stall when button released
            stallCount = 0;
            transferStalled = false;
        }

        if (transferStalled) {
            transferPower = 0;
        }
        transfer.setPower(transferPower);

        // --- Latch servo ---
        if (gamepad1.dpad_up) {
            latchPosition = LATCH_OPEN;
            latch.set(latchPosition);
        } else if (gamepad1.dpad_down) {
            latchPosition = LATCH_CLOSED;
            latch.set(latchPosition);
        }

        // Phase 3 — uncomment when turret is ready
        // if (gamepad1.dpad_left)  { turretL1.set(0.0); turretL2.set(0.0); }  // same direction
        // if (gamepad1.dpad_right) { turretL1.set(1.0); turretL2.set(1.0); }  // same direction
        // if (gamepad1.y) hood.set(0.0);
        // if (gamepad1.x) hood.set(1.0);

        // --- Telemetry ---
        telemetry.addData("Loop", loopCount);
        telemetry.addData("--- Intake ---", "");
        telemetry.addData("  Power", intakePower);
        telemetry.addData("--- Transfer ---", "");
        telemetry.addData("  Power", transferPower);
        telemetry.addData("  Current (A)", transferCurrent);
        telemetry.addData("  Stalled", transferStalled);
        telemetry.addData("--- Latch ---", "");
        telemetry.addData("  Position", latchPosition);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.set(0);
        transfer.setPower(0);
    }
}
