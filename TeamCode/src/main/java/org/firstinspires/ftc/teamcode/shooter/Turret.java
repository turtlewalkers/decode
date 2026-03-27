//package org.firstinspires.ftc.teamcode.shooter;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import com.seattlesolvers.solverslib.controller.PIDController;
//import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
//import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;
//
//@Config
//@TeleOp
//public class Turret extends OpMode {
//    private FtcDashboard dashboard;
//    public static final double TURRET_MIN = -135;
//    public static final double TURRET_MAX = 260;
//    public static double m = -123.71, b = 256.37;
//    public static double offset = 406, limit = 240;
//    private ProfiledPIDController controller;
//    private AnalogInput abs;
//    public static double p = 2.1, i = 0, d = 0.02;
//    public static double maxV = 580, maxA = 4000;
//    public static double target = 0;
//    public static double acceleration = 0;
//    public static double currentVelocity = 0, previousVelocity = 0;
//    private VoltageSensor volt;
//    private ElapsedTime timer = new ElapsedTime();
//    private DcMotorEx turret;
//    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
//
//    @Override
//    public void init() {
//        controller = new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(maxV, maxA));
////        controller = new PIDController(p, i, d);
//        dashboard = FtcDashboard.getInstance();
//        turret = hardwareMap.get(DcMotorEx.class, "turret");
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
//        abs = hardwareMap.get(AnalogInput.class, "abs");
//        timer.reset();
//    }
//
//    @Override
//    public void loop() {
//        double presentVoltage = volt.getVoltage();
//        controller.setPID(p, i, d);
//        double pos = turret.getCurrentPosition() / TICKS_PER_DEGREES;
//        double absVoltage = abs.getVoltage();
//        double absDegrees = absVoltage * m + b;
//        if (absDegrees >= limit) {
//            absDegrees -= offset;
//        }
//        double pid = controller.calculate(pos, target);
//        turret.setPower(pid / presentVoltage);
//
//        currentVelocity = turret.getVelocity() / TICKS_PER_DEGREES;
//        acceleration = (currentVelocity - previousVelocity) / timer.seconds();
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Position", pos);
//        packet.put("Target", target);
//        packet.put("Power", pid);
//        packet.put("Velocity", currentVelocity);
//        packet.put("Acceleration", acceleration);
//        packet.put("absVoltage", absVoltage);
//        packet.put("voltage plotted", absDegrees);
//        dashboard.sendTelemetryPacket(packet);
//
//        previousVelocity = currentVelocity;
//        timer.reset();
//    }
//}