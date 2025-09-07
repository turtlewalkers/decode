package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Constants {

    public static boolean useSecondaryTranslationalPIDF = false;
    public static boolean useSecondaryHeadingPIDF = false;
    public static boolean useSecondaryDrivePIDF = false;
    public static FollowerConstants followerConstants = new FollowerConstants()

            // robot config
            .mass(5)

            // sysid
            .forwardZeroPowerAcceleration(0)
            .lateralZeroPowerAcceleration(0)

            .useSecondaryTranslationalPIDF(useSecondaryTranslationalPIDF)
            .useSecondaryHeadingPIDF(useSecondaryHeadingPIDF)
            .useSecondaryDrivePIDF(useSecondaryDrivePIDF);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            // global config
            .maxPower(1)

            // motor config
            .rightFrontMotorName("rf")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)

            .rightRearMotorName("rr")
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .leftRearMotorName("lr")
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)

            // sysid
            .xVelocity(0)
            .yVelocity(0);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
