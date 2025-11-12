package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Autonomous()
public class AutonLinear extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static int target = 430;
    private int pathState;
    private static double vel = 0;
    private final Pose Start = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(60  , 85, Math.toRadians(135));
    private final Pose Grab1 = new Pose(48, 85, Math.toRadians(180));
    private final Pose Collect1 = new Pose(19, 85, Math.toRadians(180));
    private final Pose Grab2 = new Pose(48, 60, Math.toRadians(180));
    private final Pose Collect2 = new Pose(12, 60, Math.toRadians(180));
    private final Pose Grab3 = new Pose(48, 36, Math.toRadians(180));
    private final Pose Collect3 = new Pose(12, 36, Math.toRadians(180));
    private final Pose byebye = new Pose(50, 70, Math.toRadians(90));
    private final Pose byebye2 = new Pose(50, 69, Math.toRadians(90));
    private Path PreloadShoot;
    private PathChain Goto1, Pickup1, Shoot1, Goto2, Pickup2, Shoot2, Pickup3, Shoot3, Goto3, tatawireless, tatawireless2;
    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.init(hardwareMap);
        robot.hood.setPosition(0.3);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        PreloadShoot = new Path(new BezierLine(Start, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), ScorePosition.getHeading());

        Goto1 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab1))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab1.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Grab1, Collect1))
                .setLinearHeadingInterpolation(Grab1.getHeading(), Collect1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(Collect1, ScorePosition))
                .setLinearHeadingInterpolation(Collect1.getHeading(), ScorePosition.getHeading())
                .build();

        Goto2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab2))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab2.getHeading())
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(Grab2, Collect2))
                .setLinearHeadingInterpolation(Grab2.getHeading(), Collect2.getHeading())
                .build();

        Shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Collect2, ScorePosition))
                .setLinearHeadingInterpolation(Collect2.getHeading(), ScorePosition.getHeading())
                .build();

        Goto3 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, Grab3))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), Grab3.getHeading())
                .build();

        Pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(Grab3, Collect3))
                .setLinearHeadingInterpolation(Grab3.getHeading(), Collect3.getHeading())
                .build();

        Shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Collect3, ScorePosition))
                .setLinearHeadingInterpolation(Collect3.getHeading(), ScorePosition.getHeading())
                .build();

        tatawireless = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();
        tatawireless2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePosition, byebye))
                .setLinearHeadingInterpolation(ScorePosition.getHeading(), byebye.getHeading())
                .build();

        waitForStart();
        try {
            Memory.autoRan = true;
            Memory.allianceRed = false;

            robot.intake.setPower(1);
            updateShooterPID();

            robot.latch.setPosition(0);
            followPath(PreloadShoot, true);

            waitShoot(300);

            runShooter();

            followPath(Goto1, true);
            robot.intake.setPower(1);
            followPath(Pickup1, true);

            telemetry.addData("Before crash", 1);
            telemetry.update();

            //waitMillis(1000);
            followPath(Shoot1, true);

//            waitShoot(300);

            runShooter();

            followPath(Goto2, true);
            robot.intake.setPower(1);
            followPath(Pickup2, true);
            //waitMillis(200);
            robot.intake.setPower(0);

            followPath(Shoot2, true);

            waitShoot(300);

            runShooter();

            followPath(Goto3, true);
            robot.intake.setPower(1);
            followPath(Pickup3, true);
            //waitMillis(1000);
            robot.intake.setPower(0);

            followPath(Shoot3, true);

            waitShoot(300);

            runShooter();


            robot.shooterb.setPower(0);
            robot.shootert.setPower(0);
            followPath(tatawireless, true);
            Memory.robotAutoX = follower.getPose().getX();
            Memory.robotAutoY = follower.getPose().getY();
            Memory.robotHeading = follower.getPose().getHeading();
            robot.shooterb.setPower(0);
            robot.shootert.setPower(0);
            robot.turret.setPower(0);

            while (opModeIsActive()) {
                double turretPos = ((double)robot.turret.getCurrentPosition()) / TurtleRobot.TICKS_PER_DEGREES;
                telemetry.addData("Turret Pos", turretPos);

                double turretPower = robot.controllerTurret.calculate(turretPos, 0);
                telemetry.addData("Power", turretPower);
                robot.turret.setPower(turretPower);
                follower.update();
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.toString());
            telemetry.update();
            sleep(10000000);
        }

    }

    private void runShooter() {
        robot.latch.setPosition(1);
        robot.intake.setPower(1);
        waitShoot(2200);
        robot.latch.setPosition(0);
        robot.intake.setPower(0);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();
        while (follower.isBusy()) {
            follower.update();
            updateShooterPID();
            updateTurretPID();
        }
    }
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();
        while (follower.isBusy()) {
            follower.update();
            updateShooterPID();
            updateTurretPID();
        }
    }

    public void updateShooterPID() {
            robot.controller.setPID(robot.p, robot.i, robot.d);
            double presentVoltage = robot.volt.getVoltage();
            vel = vel * robot.alpha + robot.shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - robot.alpha);

            double pid = robot.controller.calculate(vel, target);
            pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
            robot.shooterb.setPower(pid * 0.9);
            robot.shootert.setPower(-0.9 * pid);
    }

    public void updateTurretPID() {
        double turretPos = ((double)robot.turret.getCurrentPosition()) / TurtleRobot.TICKS_PER_DEGREES;
        telemetry.addData("Turret Pos", turretPos);

        double turretPower = robot.controllerTurret.calculate(turretPos, 5);
        telemetry.addData("Power", turretPower);
        robot.turret.setPower(turretPower);
    }

    public void waitShoot(long sleepTimeMillis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + sleepTimeMillis) {
            updateShooterPID();
            updateTurretPID();
//            follower.update();
            sleep(1);
        }
    }

    public void waitMillis(long sleepTimeMillis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + sleepTimeMillis) {
//            follower.update();
            sleep(1);
        }
    }
}