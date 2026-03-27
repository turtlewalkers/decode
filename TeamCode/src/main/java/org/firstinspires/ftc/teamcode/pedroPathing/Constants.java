package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") // 0
            .rightRearMotorName("rb") // 1
            .leftRearMotorName("lb") // 2
            .leftFrontMotorName("lf") // 3
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(78.420)
            .yVelocity(63.420);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-31)
            .lateralZeroPowerAcceleration(-60)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.022, 0.006))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0, 0.04, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.28,0.114,0.0000027,0.6,0.007))
            .centripetalScaling(0.00034)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.2, 0.09, 0.0012));
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.25)
            .strafePodX(-6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints2 = new PathConstraints(0.995, 0, 1.4, 0.1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints2)
//                .mecanumDrivetrain(driveConstants)
                .build();
    }
}