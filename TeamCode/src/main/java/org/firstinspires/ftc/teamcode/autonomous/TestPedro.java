package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
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
public class TestPedro extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses:
    private final Pose Start = new Pose(28.5, 128, Math.toRadians(135));
    private final Pose ScorePosition = new Pose(60  , 85, Math.toRadians(135));
    private Path PreloadShoot;

    public void buildpaths() {
        PreloadShoot = new Path(new BezierLine(Start, ScorePosition));
        PreloadShoot.setLinearHeadingInterpolation(Start.getHeading(), ScorePosition.getHeading());
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start);

        buildpaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new InstantCommand(),
                new FollowPathCommand(follower, PreloadShoot, true)
        );
    }


    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }

    @Override
    public void end() {
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();
        Memory.robotHeading = follower.getPose().getHeading();
        Memory.robotPose = follower.getPose();
    }
}