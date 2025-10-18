package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class TurtleRobot {
    /* Public OpMode members. */
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public  DcMotorEx rightFront;
    public  DcMotorEx rightBack;
    public DcMotorEx leftFront;
    public  DcMotorEx leftBack;
    public DcMotorEx intake;
    public DcMotorEx shootertop;
    public DcMotorEx shooterbottom;
    public DcMotorEx turret;

    HardwareMap hwMap;
    public OpMode myOpMode;

    /* Constructor */
    public TurtleRobot(OpMode opmode) { myOpMode = opmode;}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        /**
         * Control Hub:
         * Motors:
         * 0 - rightFront
         * 1 - rightBack
         * 2 - leftBack
         * 3 - intake
         * Expansion Hub:
         * Motors:
         * 0 - leftFront
         * 1 - shootertop
         * 2 - shooterbottom
         * 3 - turret
         **/

        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "lf");
        leftBack = hwMap.get(DcMotorEx.class,   "lb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");
        rightBack = hwMap.get(DcMotorEx.class, "rb");
        intake = hwMap.get(DcMotorEx.class, "intake");
        shootertop = hwMap.get(DcMotorEx.class, "shootertop");
        shooterbottom = hwMap.get(DcMotorEx.class, "shooterbottom");
        turret = hwMap.get(DcMotorEx.class, "turret");

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        turret.setPower(0);
        intake.setPower(0);
        shootertop.setPower(0);
        shooterbottom.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }
}