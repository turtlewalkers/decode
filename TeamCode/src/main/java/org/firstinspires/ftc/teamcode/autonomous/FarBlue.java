package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous
public class FarBlue extends CommandOpMode {
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    // Poses:
    private final Pose Start = new Pose(48+6.5, 8.5, Math.toRadians(90));
    private final Pose End = new Pose(48+6.5, 25, Math.toRadians(90));
    private Path Leave;

    public void buildpaths() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        Leave = new Path(new BezierLine(Start, End));
        Leave.setLinearHeadingInterpolation(Start.getHeading(), End.getHeading());
    }

    @Override
    public void initialize() {
        super.reset();
        Memory.allianceRed = false;
        Memory.autoRan = true;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);
        intake = new Intake(hardwareMap, () -> follower, 6, 138);
        shooter = new Shooter(hardwareMap, () -> follower, 6, 138, true);

        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        intake.close(),
                        shooter.flywheel(true),
                        shooter.turretOff(false),
                        new WaitCommand(1000),
                        intake.collect(),
                        intake.open(),
                        new WaitCommand(2000),
                        new FollowPathCommand(follower, Leave)
                )
        );
    }


    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();

        Memory.robotHeading = follower.getHeading();
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotPose = follower.getPose();
    }

    @Override
    public void end() {
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();
        Memory.robotPose = follower.getPose();
        Memory.autoRan = true;

        schedule(new InstantCommand(() -> shooter.turretOff(true)));
    }
}