package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.24)
            .forwardZeroPowerAcceleration(-30.81)
            .lateralZeroPowerAcceleration(-56)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0, 0.01))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.02,0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01))
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
            .xVelocity(78.5)
            .yVelocity(64.5);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.0020289436789)
            .leftPodY(2.8937)
            .rightPodY(0)
            .strafePodX(-2.362)
            .leftEncoder_HardwareMapName("LeftBackDrive")
            .rightEncoder_HardwareMapName("LeftFrontDrive")
            .strafeEncoder_HardwareMapName("RightFrontDrive")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .turnTicksToInches(0.00212);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.4, 0.2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
