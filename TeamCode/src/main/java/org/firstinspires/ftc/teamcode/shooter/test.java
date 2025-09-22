package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class test extends OpMode {

    private DcMotor shooter, shooter2;
    private CRServo transfer1, transfer2;

    public void init() {
        shooter = hardwareMap.get(DcMotor.class, "rf");
        shooter2 = hardwareMap.get(DcMotor.class, "rr");
        transfer1 = hardwareMap.get(CRServo.class, "transfer1");
        transfer2 = hardwareMap.get(CRServo.class, "transfer2");
    }

    public void loop() {
        shooter.setPower(gamepad1.left_stick_y);
        shooter2.setPower(-gamepad1.left_stick_y);
        transfer2.setPower(gamepad1.right_stick_y);
        transfer1.setPower(-gamepad1.right_stick_y);
        telemetry.update();
    }
}
