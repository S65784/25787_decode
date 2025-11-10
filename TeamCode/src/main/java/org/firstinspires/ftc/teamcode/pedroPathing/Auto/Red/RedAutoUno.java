package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

//import android.graphics.Point;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import pedroPathing.Constants;




@Autonomous(name = "RedAutoUno", group = "Competition")
public class RedAutoUno extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Define Poses
    private final Pose startPose = new Pose(123.5, 122.5, Math.toRadians(38));
    private final Pose scorePose = new Pose(114.68, 113.07, Math.toRadians(38));
    private final Pose controlPickup1Ready = new Pose(92, 97, Math.toRadians(38));
    private final Pose pickup1Ready = new Pose(99, 83, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(131, 83, Math.toRadians(0));
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
//    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,controlPickup1Ready,pickup1Ready))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Ready.getHeading())
                .addPath(new BezierLine(pickup1Ready,pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Ready.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(scorePose), new Pose(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        park = new Path(new BezierCurve(new Point(scorePose),
//                new Point(parkControlPose),
//                new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(-1);
                }
                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
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