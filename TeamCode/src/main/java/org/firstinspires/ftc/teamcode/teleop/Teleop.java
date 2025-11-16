package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.robot.Memory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class Teleop extends OpMode {
    public static Follower follower;
    private PIDController controller, controllerTurret;
    private TelemetryManager telemetryM;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double pT = 0.1, iT = 0, dT = 0;
    public static double f = 0.0265;
    private static double vel = 0;
    public static double target = 0;
    public static double alpha = 0.6;
    InterpLUT RPM = new InterpLUT();
    InterpLUT angle = new InterpLUT();
    InterpLUT shottime = new InterpLUT();
    private DcMotorEx shooterb, shootert, intake, turret;
    private Servo hood;
    private VoltageSensor volt;
    public static double tangle = 40;
    public static double theta = 0;
    public static double shooterX = 138;
    public static double shooterY = 138;
    Servo latch;
    private double turretOffset = 0;
    private static final int TICKS_MIN = -330;
    private static final int TICKS_MAX = 990;
    public static  double TICKS_PER_DEGREES = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0 * 3.0) / 360.0;
    public boolean stopAutoTurret = false;
    @Override
    public void init() {
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
        RPM.add(0, 315);
        RPM.add(40.5, 315);
        RPM.add(60.25, 330);
        RPM.add(90, 380);
        RPM.add(106.5, 410);
        RPM.add(210, 410);
        RPM.createLUT();

        angle.add(0, 1);
        angle.add(40.5, 1);
        angle.add(60.25, 0.3);
        angle.add(90, 0.15);
        angle.add(106.5, 0.05);
        angle.add(210, 0.05);
        angle.createLUT();

        shottime.add(0, 1);
        shottime.add(40.8, 1);
        shottime.add(61.6, 0.81);
        shottime.add(87.8, 1);
        shottime.add(106.6, 1);
        shottime.add(210, 1);
        shottime.createLUT();

        latch = hardwareMap.servo.get("latch");
    }

    @Override
    public void start() {
        follower = Constants.createFollower(hardwareMap);
        if (Memory.autoRan) {
            follower.setStartingPose(new Pose(Memory.robotAutoX, Memory.robotAutoY, 0));
        } else {
            follower.setStartingPose(new Pose(72, 72, 0));
        }
        follower.setStartingPose(new Pose(90, 72, 0));
        follower.startTeleOpDrive(true);
        follower.update();
        controller = new PIDController(p, i, d);
        Memory.autoRan = false;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            Memory.allianceRed = true;
        } else if (gamepad1.b) {
            Memory.allianceRed = false;
        }
        telemetry.addData("Alliance", Memory.allianceRed ? "Red" : "Blue");

        if (Memory.allianceRed) {
            shooterY = 6;
        } else {
            shooterY = 138;
        }
    }

    @Override
    public void loop() {
        double multiplier = 1;
        if (gamepad1.left_trigger != 0) multiplier = 0.3;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * multiplier, -gamepad1.left_stick_x * multiplier, -gamepad1.right_stick_x * multiplier, true);
        follower.update();

        if (gamepad1.dpad_up) {
            follower.setStartingPose(new Pose(72, 72, 0));
        }

        if (gamepad1.xWasPressed()) {
            stopAutoTurret = !stopAutoTurret;
        }
        latch.setPosition(gamepad1.y ? 1 : 0);

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();

        double dx = shooterX - robotX;
        double dy = shooterY - robotY;
        double distance = Math.sqrt(dx*dx + dy*dy);

        for (int i = 0; i < 5; ++i) {
            double shotTime = shottime.get(distance);

            double vX = follower.getVelocity().getXComponent();
            double vY = follower.getVelocity().getYComponent();

            dx = shooterX - robotX - vX * shotTime;
            dy = shooterY - robotY - vY * shotTime;
            distance = Math.sqrt(dx*dx + dy*dy);
        }

        telemetry.addData("vX", follower.getVelocity().getXComponent());
        telemetry.addData("vY", follower.getVelocity().getYComponent());
        telemetry.addData("Shot time", shottime.get(distance));
        telemetry.addData("shooterX", shooterX - follower.getVelocity().getXComponent() * shottime.get(distance));
        telemetry.addData("shooterY", shooterY - follower.getVelocity().getYComponent() * shottime.get(distance));

        double targetAngleRad = Math.atan2(dy, dx);
        double targetAngleDeg = Math.toDegrees(targetAngleRad) - Math.toDegrees(robotHeading);

        if (turretOffset <= 45 && turretOffset >= -45) {
            if (gamepad1.dpad_right && turretOffset > -45) {
                turretOffset -= 1;
            }
            if (gamepad1.dpad_left && turretOffset < 45) {
                turretOffset += 1;
            }
        }
        telemetry.addData("Target Angle", targetAngleDeg);
        targetAngleDeg += turretOffset;
        telemetry.addData("TurretOffset", turretOffset);
        targetAngleDeg = Math.max(targetAngleDeg, -100);
        targetAngleDeg = Math.min(targetAngleDeg, 240);
        double turretPos = ((double)turret.getCurrentPosition()) / TICKS_PER_DEGREES;
        telemetry.addData("Turret Pos", turretPos);

//        targetTicks = Math.max(TICKS_MIN, Math.min(TICKS_MAX, targetTicks));
//        turretOffset += (gamepad2.right_trigger - gamepad2.left_trigger) * 5;
//        int offsetTicks = (int)(turretOffset * TICKS_PER_DEGREE);
//        int finalTargetTicks = targetTicks + offsetTicks;
        double turretPower = controllerTurret.calculate(turretPos, targetAngleDeg);
        telemetry.addData("TurretPower", turretPower);
        telemetry.addData("stopAutoTurret", stopAutoTurret);

        if (!stopAutoTurret) {
            turret.setPower(turretPower);
        }
        intake.setPower(gamepad1.right_trigger);

        if (distance >0 && distance < 180) {
            target = RPM.get(distance);
            hood.setPosition(angle.get(distance));
        }
        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        vel = shooterb.getVelocity() * (2 * Math.PI / 28);
        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        if (!gamepad1.a || robotX >= 40) {
            shooterb.setPower((-1) * (pid + f * target) / presentVoltage);
            shootert.setPower((-1) * (pid + f * target) / presentVoltage);
        } else {
            shootert.setPower(0);
            shooterb.setPower(0);
        }
        if (gamepad1.dpad_down) {
            intake.setPower(-1);
        }

//        if (gamepad1.dpad_left)
//        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Distance: ", distance);
        telemetry.addData("x: ", robotX);
        telemetry.addData("y: ", robotY);
        telemetry.addData("Heading", Math.toDegrees(robotHeading));
        telemetry.addData("RPM: ", RPM.get(distance));
        telemetry.addData("Angle: ", angle.get(distance));
        telemetry.update();
    }
}