package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.24)
            .forwardZeroPowerAcceleration(-29)
            .lateralZeroPowerAcceleration(-62)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, .000001, .001, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.001,0.02))

            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0.01, 0.001, 0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.0001,0.0000001,0.6,0.0000))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01))
            .centripetalScaling(0.001)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontDrive")
            .rightRearMotorName("RightBackDrive")
            .leftRearMotorName("LeftBackDrive")
            .leftFrontMotorName("LeftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.3)
            .yVelocity(64);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(-2.2975)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 0.1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
