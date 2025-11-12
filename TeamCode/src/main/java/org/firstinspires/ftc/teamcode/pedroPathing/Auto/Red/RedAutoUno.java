package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

//import android.graphics.Point;



import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;


@Autonomous(name = "RedAutoUno", group = "Competition")
public class RedAutoUno extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static final double MOTOR_TICK_COUNT = 28;
    public static int TARGET_RPM = 0;
    public static int ERROR_RANGE = 100;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static double getPointPreX = 90;
    public static double getPointX = 125;
    public static double Point1Y = 83;
    public static double Point2Y = 60;
    public static double Point3Y = 35;



    // Define Poses
    private final Pose startPose = new Pose(123.5, 122.5, Math.toRadians(38));

    private final Pose scorePose = new Pose(114.68, 113.07, Math.toRadians(38));

    private final Pose controlPickup1Ready = new Pose(92, 97, Math.toRadians(38));
    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(getPointX, Point1Y, Math.toRadians(0));

    private final Pose controlPickup2Ready = new Pose(76.6, 61.9, Math.toRadians(0));
    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(getPointX, Point2Y, Math.toRadians(0));

    private final Pose controlPickup3Ready = new Pose(79, 70, Math.toRadians(0));
    private final Pose pickup3Ready = new Pose(getPointPreX, Point3Y, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(getPointX, Point3Y, Math.toRadians(0));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload = new Path(new BezierLine(startPose, startPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPickup1Ready,pickup1Ready))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Ready.getHeading())
                .setTimeoutConstraint(200)

                .addPath(new BezierLine(pickup1Ready,pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Ready.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPickup2Ready,pickup2Ready))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Ready.getHeading())
                .setTimeoutConstraint(100)
                .addPath(new BezierLine(pickup2Ready,pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,controlPickup3Ready,pickup3Ready))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Ready.getHeading())
                .setTimeoutConstraint(100)
                .addPath(new BezierLine(pickup3Ready,pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Ready.getHeading(), pickup3Pose.getHeading())
                .build();

//        park = new Path(new BezierCurve(new Point(scorePose),
//                new Point(parkControlPose),
//                new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                    follower.followPath(scorePreload);
                    sleep(100);
                    //Algorithm.Shoot(1600,50,true);
                    setPathState(-1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Algorithm.Draw(true);
                    follower.followPath(grabPickup1, true);
                    sleep(100);
                    Algorithm.Draw(false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    sleep(100);
                    Algorithm.Shoot(1600,50,true);
                    setPathState(-1);
                }
                break;



            case 3:
                if (!follower.isBusy()) {
                    Algorithm.Draw(true);
                    follower.followPath(grabPickup2, true);
                    sleep(100);
                    Algorithm.Draw(false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    sleep(100);
                    Algorithm.Shoot(1600,50,true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Algorithm.Draw(true);
                    follower.followPath(grabPickup3, true);
                    sleep(100);
                    Algorithm.Draw(false);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    sleep(100);
                    Algorithm.Shoot(1600,50,true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }


}