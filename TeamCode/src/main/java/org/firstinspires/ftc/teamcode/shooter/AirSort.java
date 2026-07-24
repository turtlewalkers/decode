/*
package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.seattlesolvers.solverslib.controller.PIDController;

@Config
@TeleOp(name = "AirSortTeleOp")
public class AirSort extends OpMode {

    // ---------- CONFIG ----------
    public static double p = 1, i = 0.1, d = 0;
    public static double kV = 0.0212;
    public static double targetRPM = 300;

    // hood positions for airsort
    public static double HOOD_UP = 0.3;
    public static double HOOD_STRAIGHT = 0.95;

    public static double SHOT_DELAY = 275;   // ms between balls

    // ---------- Devices ----------
    private DcMotorEx shooterb, shootert, intakeMotor;
    private Servo hood, intakeGate;
    private VoltageSensor volt;

    // ---------- PID ----------
    private PIDController controller;
    private double velocity;

    // ---------- Airsort FSM ----------
    private boolean sorting = false;
    private int step = 0;
    private ElapsedTime timer = new ElapsedTime();

    // ---------- Dashboard ----------
    private FtcDashboard dashboard;

    @Override
    public void init() {

        // Hardware
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        intakeGate = hardwareMap.get(Servo.class, "latch");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // PID
        controller = new PIDController(p, i, d);

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("AirSort TeleOp READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ----------------------------------------------------------
        //  ALWAYS RUN SHOOTER PID
        // ----------------------------------------------------------
        double presentVoltage = volt.getVoltage();
        double vel = shooterb.getVelocity() * (2 * Math.PI / 28);
        double flywheelPID = controller.calculate(vel, targetRPM);
        flywheelPID = Math.max(-presentVoltage, Math.min(flywheelPID, presentVoltage));
        double pidVolts = flywheelPID;
        double ffvolts = kV * targetRPM;
//        ffvolts += kS * Math.signum(target);
        double flywheelVolts = pidVolts + ffvolts;
        flywheelVolts = Math.max(-presentVoltage, Math.min(flywheelVolts, presentVoltage));
        shootert.setPower((-1) * flywheelVolts / presentVoltage);
        shooterb.setPower(flywheelVolts / presentVoltage);

        // ----------------------------------------------------------
        // BUTTON STARTS AIRSORT SEQUENCE
        // ----------------------------------------------------------

        if (gamepad1.y && !sorting) {
            sorting = true;
            step = 0;
            timer.reset();
            hood.setPosition(HOOD_UP);

            intakeMotor.setPower(1);   // pull balls up
        }

        // ----------------------------------------------------------
        // AIRSORT STATE MACHINE
        // ----------------------------------------------------------

        if (sorting) {
            switch (step) {
                case 0:
                    intakeGate.setPosition(1);
                    // First: UP shot
                    hood.setPosition(HOOD_UP);

                    if (timer.milliseconds() > SHOT_DELAY) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 1:
                    // Second: STRAIGHT shot
                    hood.setPosition(HOOD_STRAIGHT);

                    if (timer.milliseconds() > SHOT_DELAY) {
                        step++;
                        timer.reset();
                    }
                    break;

                case 2:
                    // Third: STRAIGHT shot
                    hood.setPosition(HOOD_STRAIGHT);

                    if (timer.milliseconds() > SHOT_DELAY) {
                        sorting = false;
                        intakeGate.setPosition(0);    // close gate
                    }
                    break;
            }

            // Intake automatically keeps running during sequence
            intakeMotor.setPower(1);
        }
        else {
            // Manual intake control if NOT sorting
            intakeMotor.setPower(gamepad1.right_trigger);
        }

        // ----------------------------------------------------------
        // DASHBOARD TELEMETRY
        // ----------------------------------------------------------

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity", vel);
        packet.put("Target RPM", targetRPM);
        packet.put("Sorting?", sorting);
        packet.put("Step", step);
        dashboard.sendTelemetryPacket(packet);
    }
}
*/
