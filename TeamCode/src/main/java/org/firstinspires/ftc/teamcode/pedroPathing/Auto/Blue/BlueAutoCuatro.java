package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "蓝色离开", group = "Competition")
public class BlueAutoCuatro extends OpMode {
    private Algorithm Algorihthm;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static int san = 2400;
    public static double getPointPreX = 95;
    public static double getPointX = 126.5;
    public static double Point1Y = 82;
    public static double Point2Y = 58;
    public static double Point3Y = 34.5;


    // Define Poses
    private final Pose startPose = new Pose(140-85, 140-8.4, Math.toRadians(90));

    private final Pose controlScorePose = new Pose(93.06631989596879, 62.91807542262679, Math.toRadians(0));
    private final Pose leavePose = new Pose(140-109, 140-8.4, Math.toRadians(90));

    private final Pose controlScorePose2 = new Pose(83.51625487646294, 62.73081924577374, Math.toRadians(40));
    private final Pose scorePose2 = new Pose(87.2, 81.8, Math.toRadians(40));

    private final Pose controlScorePose3 = new Pose(84, 55, Math.toRadians(40));
    private final Pose scorePose3 = new Pose(87.2, 81.87, Math.toRadians(40));


    private final Pose pickup2Ready = new Pose(103.17815344603382, 59.36020806241872, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(getPointX, 59.36020806241872, Math.toRadians(0));

    private final Pose controlTheGate = new Pose(119.84395318595578, 68.34850455136541, Math.toRadians(0));
    private final Pose theGate = new Pose(129.01950585175553, 68.16124837451235, Math.toRadians(0));

    private final Pose pickup3Ready = new Pose(101.30559167750326, Point3Y, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(131, Point3Y, Math.toRadians(0));

    private Path scorePreload, runto2, runto3, park;
    private PathChain grabPickup2, grabPickup3, scorePickup2, scorePickup3, runTheGate;

    public void buildPaths() {
//ichi ni san shi/yon go roku shichi/nana hachi kyu jyu
        scorePreload = new Path(new BezierLine(startPose, leavePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading());

//        runto2 = new Path(new BezierLine(scorePose, pickup2Ready));
//        runto2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Ready.getHeading());
//
//        runto3 = new Path(new BezierLine(scorePose, pickup3Ready));
//        runto3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Ready.getHeading());

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .build();
        runTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlTheGate, theGate))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Ready, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Ready.getHeading(), pickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, controlScorePose3, scorePose3))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(-1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootTime(true, true,2000);
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(runto2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.37);
                    Algorithm.draw();
                    follower.followPath(grabPickup2, true);
                    setPathState(40);
                }
                break;

            case 40:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(runTheGate, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootTime(true, true,2000);
                    setPathState(6);
                }
                break;


            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(runto3);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.37);
                    Algorithm.draw();
                    follower.followPath(grabPickup3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(scorePickup3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootTime(true, true,2000);
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
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Running");
        telemetry.addData("目标 RPM", Algorithm.targetRPM);
        telemetry.addData("当前 RPM", "%.2f", Algorithm.getCurrentRPM());
        telemetry.addData("test", Algorithm.test);


        telemetry.update();
        telemetry.update();
    }

    @Override
    public void init() {
        Algorihthm = new Algorithm(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        setPathState(0);

        Algorithm.servoControl();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

}


