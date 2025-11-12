package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Memory;

public class Drivetrain extends SubsystemBase {
    public static Follower follower;
    private boolean teleop = false;

    public Drivetrain(final HardwareMap hMap, boolean teleop, Pose start) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(start);
        this.teleop = teleop;
        if (teleop) follower.startTeleOpDrive();
        follower.update();
    }

    @Override
    public void periodic() {
        Memory.robotHeading = follower.getHeading();
        Memory.robotAutoX = follower.getPose().getX();
        Memory.robotAutoY = follower.getPose().getY();

        follower.update();
    }
}
