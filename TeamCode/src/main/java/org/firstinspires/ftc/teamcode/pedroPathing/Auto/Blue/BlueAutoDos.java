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

@Autonomous(name = "蓝色远端合作(施工中)", group = "Competition")//远端人玩区尝试 //完毕
public class BlueAutoDos extends OpMode {
    private Algorithm Algorihthm;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final int millitime = 10000;
    private static final double lowMaxPower = 0.6;
    private static final double shooterPower = 0.6;
    private static final double t = 0.3;
    private static final double PATH_TIMEOUT = 4000;

    // Define Poses
    private final Pose startPose = new Pose(140-84.8, 8.4, Math.toRadians(90));
    private final Pose scorePose = new Pose(140-91.18072289156626, 14.65060240963856, Math.toRadians(180-72));
    private final Pose controlPickup4Ready = new Pose(140-103.71084337349397, 4.2409638554216915, Math.toRadians(180+22));
    private final Pose pickup4Ready = new Pose(140-129.34939759036143, 19, Math.toRadians(180+22));
//    private final Pose controlPickup4Pose_1 = new Pose(122.21686746987952, 10.987951807228908, Math.toRadians(0));
//    private final Pose controlPickup4Pose_2 = new Pose(135.32530120481928, 15.807228915662655, Math.toRadians(0));
    private final Pose controlPickup4Pose_3 = new Pose(140-122.98795180722891, 22.361445783132538, Math.toRadians(180));
    private final Pose controlPickup4Pose_4 = new Pose(140-121.25301204819277, 13.108433734939762, Math.toRadians(180));
    private final Pose pickup4Pose = new Pose(140-130, 11, Math.toRadians(180+10));

//    private final Pose controlPickup4Pose1 = new Pose(124.91566265060241, 15.807228915662655, Math.toRadians(-45));
//    private final Pose pickup4Pose1 = new Pose(129.1566265060241, 10.216867469879517, Math.toRadians(-42));

    private final Pose controlScorePose4 = new Pose(140-104.09638554216868, 19.46987951807229, Math.toRadians(180-72));
    private final Pose scorePose4 = new Pose(140-92.5, 15, Math.toRadians(180-63));


    private final Pose end = new Pose(140-105.4, 14.5, Math.toRadians(180-63));

    private Path scorePreload,runto4;
    private PathChain grabPickup4, scorePickup4, grabPickup41, endpath;

    public void buildPaths() {
                                //ichi ni san shi/yon go roku shichi/nana hachi kyu jyu
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        runto4 = new Path(new BezierCurve(scorePose,controlPickup4Ready,pickup4Ready));
        runto4.setLinearHeadingInterpolation(scorePose.getHeading(), pickup4Ready.getHeading());
        grabPickup4 = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup4Ready,controlPickup4Pose_1,controlPickup4Pose_2,controlPickup4Pose_3,controlPickup4Pose_4,pickup4Pose))
                .addPath(new BezierCurve(pickup4Ready,controlPickup4Pose_3,controlPickup4Pose_4,pickup4Pose))
                .setLinearHeadingInterpolation(pickup4Ready.getHeading(), pickup4Pose.getHeading())
                 .build();
//        grabPickup41 = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup4Pose,controlPickup4Pose1,pickup4Pose1))
//                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), pickup4Pose1.getHeading())
//                .addParametricCallback(0.2, () -> Algorithm.preShooterMove(130,0.63))
//                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose, controlScorePose4, scorePose4))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), scorePose4.getHeading())
                .addParametricCallback(0.21, () -> Algorithm.keep())

                .build();// 🫥

        endpath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose4, end))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), end.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                Algorithm.shootMode4.preShoot();;
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Algorithm.shootMode4.shootCheckOnceTime(millitime);
                    setPathState(2);
                }
                break;



            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(runto4);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathTimeout(5);
                    follower.setMaxPower(lowMaxPower);
                    Algorithm.draw();
                    follower.followPath(grabPickup4, true);
                    setPathState(5);
                }
                break;

//            case 50:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabPickup41, true);
//                    Algorithm.keep();
//                    setPathState(5);
//                }
//                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.keep();
                    Algorithm.stopShoot();
                    setPathState(60);
                }
                break;

            case 60:
                if (!follower.isBusy()) {
                    Algorithm.shootMode4.preShoot();
                    follower.followPath(scorePickup4, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    Algorithm.shootMode4.shootCheckOnceTime(millitime);
                    setPathState(7);
                }
                break;

            case 7:
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
    private boolean pathTimeout(int nextState) {
        if (pathTimer.getElapsedTime() > PATH_TIMEOUT) {
            follower.breakFollowing();   // 强制停止路径
            setPathState(nextState);     // 直接跳到下一个安全状态
            return true;
        }
        return false;
    }



}