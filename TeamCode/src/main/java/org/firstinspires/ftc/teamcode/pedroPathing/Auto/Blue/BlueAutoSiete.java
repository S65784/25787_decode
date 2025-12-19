package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;

@Autonomous(name = "12527coop蓝-近端(撞gate*n)", group = "Competition")
public class BlueAutoSiete extends OpMode {
    private Algorithm Algorihthm;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final int millitime = 1600;
    private static final double lowMaxPower = 0.657;
    private static final double t = 0.5;


    private static final double getPointPreX = 45;
    private static final double getPointX = 12;
    private static final double Point1Y = 82-0.6;
    private static final double Point2Y = 58-0.6;



    // Define Poses
    private final Pose startPose = new Pose(16.5, 122.5, Math.toRadians(142));
    private final Pose scorePose = new Pose(40, 99.85542168674698, Math.toRadians(180-38));

    private final Pose controlPickup1Ready = new Pose(48, 97, Math.toRadians(180));
    private final Pose pickup1Ready = new Pose(46.45783132530121, 84.43373493975903, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(16.19277108433735, 84.43373493975903, Math.toRadians(180));

    private final Pose controlTheGate = new Pose(36.81927710843374, 82.89156626506025, Math.toRadians(90));
    private final Pose theGate = new Pose(12, 72, Math.toRadians(90));

    private final Pose controlScorePose1 = new Pose(25.25301204819277, 81.92771084337349, Math.toRadians(32));
    private final Pose scorePose1 = new Pose(40, 99.85542168674698, Math.toRadians(180-35.6));

    private final Pose controlPickup2Ready = new Pose(63.4, 61.9, Math.toRadians(180));
    private final Pose pickup2Ready = new Pose(getPointPreX, 59.951807228915655, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(getPointX, 59.951807228915655, Math.toRadians(180));

    private final Pose controlScorePose2 = new Pose(36, 60.3, Math.toRadians(148));
    private final Pose scorePose2 = new Pose(40, 99.85542168674698, Math.toRadians(180-35));


    private final Pose end = new Pose(140-120.78023407022106, 93.6, Math.toRadians(180-35));
    private Path scorePreload, runto1, runto2;
    private PathChain runTheGate, grabPickup1, grabPickup2, scorePickup1, scorePickup2, endpath;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        runto1 = new Path(new BezierCurve(scorePose,controlPickup1Ready,pickup1Ready));
        runto1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Ready.getHeading());

        runto2 = new Path(new BezierCurve(scorePose1,controlPickup2Ready,pickup2Ready));
        runto2.setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Ready.getHeading());


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Ready,pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Ready.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.26, () -> Algorithm.preShooterMove(700,0.63))
                .build();
        runTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,controlTheGate, theGate))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), theGate.getHeading())
                .addParametricCallback(0.21, () -> Algorithm.keep())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(theGate, controlScorePose1, scorePose1))
                .setLinearHeadingInterpolation(theGate.getHeading(), scorePose1.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.23, () -> Algorithm.preShooterMove(900,0.63))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();



        endpath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, end))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), end.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                Algorithm.shootMode2.preShoot();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootCheckOnceTime(millitime);
                    setPathState(2);
                }
                break;



            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(runto1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(lowMaxPower);
                    Algorithm.draw();
                    follower.followPath(grabPickup1, true);
                    setPathState(30);
                }
                break;


            case 30:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.keep();
                    Algorithm.stopShoot();

                    Algorithm.sleep(3000);
                    follower.followPath(runTheGate, true);
                    Algorithm.sleepForAWhile(400);//450
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.preShoot();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootCheckOnceTime(millitime);
                    setPathState(6);
                }
                break;



            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(runto2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    Algorithm.draw();
                    follower.followPath(grabPickup2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    setPathState(90);
                }
                break;

            case 90:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.preShoot();
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootCheckOnceTime(millitime);
                    setPathState(10);
                }
                break;



            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(endpath, true);
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