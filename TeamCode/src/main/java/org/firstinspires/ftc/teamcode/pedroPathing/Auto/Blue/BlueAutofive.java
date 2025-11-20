package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
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

@Autonomous(name = "蓝色远端合作5", group = "Competition")
public class BlueAutofive extends OpMode {
    private Algorithm Algorihthm;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static int san = 2400;
    public static double getPointPreX = 140-95;
    public static double getPointX = 140-126.5;
    public static double Point1Y = 82;
    public static double Point2Y = 58;
    public static double Point3Y = 34.5;


    // Define Poses
    private final Pose startPose = new Pose(59.49116669367494, 8.384279475982526, Math.toRadians(90));

    private final Pose controlGatePose = new Pose(74.20146885501245, 45.69432314410481, Math.toRadians(180));
    private final Pose gatePose = new Pose(51.77352125675917, 74.2008733624454, Math.toRadians(180));
    private final Pose reachGatePose = new Pose(14.882691562436031, 74.2008733624454, Math.toRadians(180));

    private final Pose controlScorePose = new Pose(7.336840034051754, 128.27947598253274, Math.toRadians(180));
    private final Pose scorePose = new Pose(53.24077016505612, 136.66375545851528, Math.toRadians(180));

    private final Pose controlScorePose3 = new Pose(140-84, 55, Math.toRadians(180-40));
    private final Pose scorePose3 = new Pose(140-87.2, 81.87, Math.toRadians(180-40));


    //un dux toi
    private final Pose pickup2Ready = new Pose(140-103.17815344603382, 59.36020806241872, Math.toRadians(180-0));
    private final Pose pickup2Pose = new Pose(140-getPointX, 59.36020806241872, Math.toRadians(180-0));

//    private final Pose controlTheGate = new Pose(140-119.84395318595578, 68.34850455136541, Math.toRadians(180-0));
//    private final Pose theGate = new Pose(140-129.01950585175553, 68.16124837451235, Math.toRadians(180-0));

    private final Pose pickup3Ready = new Pose(140-101.30559167750326, Point3Y, Math.toRadians(180-0));
    private final Pose pickup3Pose = new Pose(140-131, Point3Y, Math.toRadians(180-0));

    private Path scorePreload, runto2, runto3, park;
    private PathChain grabPickup2, grabPickup3, scorePickup2, scorePickup3, runTheGate1,runTheGate2;

    public void buildPaths() {
//ichi ni san shi/yon go roku shichi/nana hachi kyu jyu
        scorePreload = new Path(new BezierCurve(reachGatePose, controlScorePose, scorePose));
        scorePreload.setLinearHeadingInterpolation(reachGatePose.getHeading(), scorePose.getHeading());

        runto2 = new Path(new BezierLine(scorePose,pickup2Ready));
        runto2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Ready.getHeading());

        runto3 = new Path(new BezierLine(scorePose,pickup3Ready));
        runto3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Ready.getHeading());

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready,pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .build();
        runTheGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,controlGatePose, gatePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), gatePose.getHeading())
                .build();
        runTheGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, reachGatePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), reachGatePose.getHeading())
                .build();
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
//                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Ready,pickup3Pose))
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
                follower.followPath(runTheGate1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(runTheGate2);
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(1750, Algorithm.ERROR_RANGE_SAN, true, 4000);
                    setPathState(-3);
                }
                break;

//            case 3:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(0.37);
//                    Algorithm.draw();
//                    follower.followPath(grabPickup2, true);
//                    setPathState(40);
//                }
//                break;
//
//            case 40:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    Algorithm.stopShoot();
//                    follower.followPath(runTheGate, true);
//                    setPathState(4);
//                }
//                break;

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
                    Algorithm.shootTime(san, Algorithm.ERROR_RANGE_SAN, true, 4000);
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
                    Algorithm.shootTime(san, Algorithm.ERROR_RANGE_SAN, true, 4000);
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
        telemetry.addData("test",Algorithm.test);


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