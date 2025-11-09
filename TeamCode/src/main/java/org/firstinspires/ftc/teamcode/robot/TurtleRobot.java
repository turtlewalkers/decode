package org.firstinspires.ftc.teamcode.robot;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class TurtleRobot {
    public OpMode myOpMode;
    public static Follower follower;
    public PIDController controller, controllerTurret;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.12, iT = 0, dT = 0;
    public static double f = 0.0265;
    public static double target = 0;
    public static double vel = 0;
    public static double alpha = 0.6;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    public DcMotorEx shooterb, shootert, intake, turret;
    public Servo hood;
    public VoltageSensor volt;
    public static double tangle = 40;
    public static double theta = 0;
    public static double shooterX = 135;
    public static double shooterY = 135;
    public Servo latch;
    public static double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;

    public TurtleRobot(OpMode opmode) { myOpMode = opmode;
    }

    public void init(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        controllerTurret = new PIDController(pT, iT, dT);
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
//        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        RPM.add(20, 350);
        RPM.add(39, 350);
        RPM.add(50, 375);
        RPM.add(60, 400);
        RPM.add(74, 415);
        RPM.add(90, 450);
        RPM.add(180, 450);
        RPM.createLUT();

        angle.add(20, 1);
        angle.add(39, 0.7);
        angle.add(50, 0.6);
        angle.add(60, 0.5);
        angle.add(74, 0.4);
        angle.add(90, 0.3);
        angle.add(180, 0.3);
        angle.createLUT();

        latch = hardwareMap.servo.get("latch");
    }
}
