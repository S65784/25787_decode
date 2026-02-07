
package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

import static android.os.SystemClock.sleep;

import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.pedroPathing.TurretAlgorithm;

@Autonomous(name = "红(推gate*2)", group = "Competition")
public class RedAutoCinco extends OpMode {
    private Algorithm Algorihthm;
    private TurretAlgorithm turretAlgorithm;
    private TelemetryManager telemetryManager;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final int millitime = 925;//1400
    private static final double lowMaxPower = 0.7;
    private static final double t = 0.3;
    private static final double PATH_TIMEOUT = 5000;


    private static final double getPointPreX = 99;
    private static final double getPointX = 126.5;
    private static final double Point1Y = 82-0.6;
    private static final double Point2Y = 59.1807228915655;
    private static final double Point3Y = 34.5-0.6;


    // Define Poses
    private final Pose startPose = new Pose(123.5, 122.5, Math.toRadians(38));
    private final Pose scorePose = new Pose(100, 99.8, Math.toRadians(38));//

    private final Pose controlPickup1Ready = new Pose(93.94464033850494, 96.27080394922427, Math.toRadians(0));
    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(128.9, Point1Y, Math.toRadians(0));

    private final Pose controlTheGate = new Pose(104.93227091633466, 69.4183266932271, Math.toRadians(270));//114.93227091633466
    private final Pose theGate = new Pose(140-7.5, 72, Math.toRadians(0));

    private final Pose controlScorePose1 = new Pose(140-25.25301204819277, 81.92771084337349, Math.toRadians(35.6));
    private final Pose scorePose1 = new Pose(140-40, 99.8, Math.toRadians(37.2));//

    private final Pose controlPickup2Ready = new Pose(140-54.24097984598117, 70.10588235294118, Math.toRadians(0));
    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(129, Point2Y, Math.toRadians(0));

    private final Pose controlTheGate2 = new Pose(140-34, 52.91444600280506, Math.toRadians(270));//140-20
    private final Pose theGate2 = new Pose(140-7.2, 72, Math.toRadians(0));


    private final Pose controlScorePose2 = new Pose(104, 60.3, Math.toRadians(32));
    private final Pose scorePose2 = new Pose(100, 99.8, Math.toRadians(36.5));//

    private final Pose controlPickup3Ready = new Pose(140-50.41176470588236, 67.45882352941175, Math.toRadians(0));
    private final Pose pickup3Ready = new Pose(getPointPreX, Point3Y, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(131.3, Point3Y, Math.toRadians(0));

    private final Pose controlScorePose3 = new Pose(102, 39, Math.toRadians(32));
    private final Pose scorePose3 = new Pose(99, 99.8, Math.toRadians(35));//

    private final Pose end = new Pose(96, 108, Math.toRadians(0));

    private Path scorePreload, runto1, runto2, runto3;
    private PathChain runTheGate, runTheGate2, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, endpath;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        runto1 = new Path(new BezierCurve(scorePose,controlPickup1Ready,pickup1Ready));
        runto1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Ready.getHeading());

        runto2 = new Path(new BezierCurve(scorePose,controlPickup2Ready,pickup2Ready));
        runto2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Ready.getHeading());

        runto3 = new Path(new BezierCurve(scorePose,controlPickup3Ready,pickup3Ready));
        runto3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Ready.getHeading());


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Ready,pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Ready.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.03, () -> Algorithm.reverseBlender(-1))
                .build();
        runTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,controlTheGate, theGate))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), theGate.getHeading())
      //          .addParametricCallback(0.21, () -> Algorithm.keep())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(theGate, controlScorePose1, scorePose1))
                .setLinearHeadingInterpolation(theGate.getHeading(), scorePose1.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.03, () -> Algorithm.reverseBlender(-1))
                .build();
        runTheGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,controlTheGate2, theGate2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), theGate2.getHeading())
                //.addParametricCallback(0.21, () -> Algorithm.keep())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(theGate2, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(theGate2.getHeading(), scorePose2.getHeading())
               // .addParametricCallback(0.13, () -> Algorithm.keep())
                .build();


        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Ready,pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Ready.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.03, () -> Algorithm.reverseBlender(-1))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, controlScorePose3, scorePose3))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
 //               .addParametricCallback(0.13, () -> Algorithm.keep())
                .build();

        endpath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose3, end))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), end.getHeading())
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
                    follower.setMaxPower(1);
                    setPathState(30);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                   // follower.setMaxPower(1);
                    Algorithm.keep();
                    follower.followPath(runTheGate, true);
                    Algorithm.stopShoot();
                    Algorithm.sleepForAWhile(210);//450 1200
                    setPathState(4);
                }
                pathTimeout(5000,4);
                break;



            case 4:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.preShoot();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                pathTimeout(5000,4);
                break;

            case 5:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootCheckOnceTime(millitime);
                    setPathState(6);
                }
                pathTimeout(8000,4);
                break;



            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(runto2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    Algorithm.draw();
                    follower.followPath(grabPickup2, true);
                    setPathState(8);
                }
                break;


            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.keep();
                    follower.followPath(runTheGate2, true);
                    Algorithm.stopShoot();
                    Algorithm.sleepForAWhile(300);//450 360
                    setPathState(90);
                }
                pathTimeout(3000,90);
                break;

            case 90:
                if (!follower.isBusy()) {
                    Algorithm.sleepForAWhile(203);//2207
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
                    follower.followPath(runto3);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.45);
                    Algorithm.draw();
                    follower.followPath(grabPickup3, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();

                    setPathState(130);
                }
                break;

            case 130:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.preShoot();;
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    Algorithm.shootMode2.shootCheckOnceTime(millitime);
                    setPathState(14);
                }
                break;

            case 14:
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
        Algorithm.updateRPM();

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

//        telemetryManager.addData("test",Algorithm.test);
//        telemetryManager.update(telemetry);
    }

    @Override
    public void init() {
        Algorihthm = new Algorithm(hardwareMap);
        turretAlgorithm = new TurretAlgorithm(hardwareMap,telemetry,Algorithm.Alliance.RED,follower);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        setPathState(0);

        Algorithm.servoControl();
        turretAlgorithm.setCenter();
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

    private boolean pathTimeout(int maxtime, int nextState) {
        if (pathTimer.getElapsedTime() > maxtime) {
            follower.breakFollowing();   // 强制停止路径
            setPathState(nextState);     // 直接跳到下一个安全状态
            return true;
        }
        return false;
    }


}