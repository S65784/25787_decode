package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

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

@Autonomous(name = "红色近端单独跑", group = "Competition")
public class RedAutoUno extends OpMode {
    private Algorithm Algorihthm;
    private final ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final int millitime = 1800;
    private static final double lowMaxPower = 0.657;
    private static final double t = 0.5;


    private static final double getPointPreX = 93.9;
    private static final double getPointX = 126.5;
    private static final double Point1Y = 82;
    private static final double Point2Y = 58;
    private static final double Point3Y = 34.5;


    // Define Poses
    private final Pose startPose = new Pose(123.5, 122.5, Math.toRadians(38));

    private final Pose scorePose = new Pose(100, 99.8, Math.toRadians(38));
    private final Pose scorePose1 = new Pose(100, 99.8, Math.toRadians(35.6));
    private final Pose controlScorePose2 = new Pose(104, 60.3, Math.toRadians(35.6));
    private final Pose scorePose2 = new Pose(100, 99.8, Math.toRadians(35));
    private final Pose controlScorePose3 = new Pose(113, 39, Math.toRadians(35));
    private final Pose scorePose3 = new Pose(100, 99.8, Math.toRadians(35));


    private final Pose controlPickup1Ready = new Pose(93.94464033850494, 96.27080394922427, Math.toRadians(0));
    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(getPointX, Point1Y, Math.toRadians(0));

    private final Pose controlPickup2Ready = new Pose(76.6, 61.9, Math.toRadians(0));
    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(getPointX, Point2Y, Math.toRadians(0));

    private final Pose controlPickup3Ready = new Pose(84, 55, Math.toRadians(0));
    private final Pose pickup3Ready = new Pose(getPointPreX, Point3Y, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(131, Point3Y, Math.toRadians(0));
    private final Pose end = new Pose(120.78023407022106, 93.6, Math.toRadians(32));


    private Path scorePreload, runto1, runto2, runto3, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, endpath;

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
                .addParametricCallback(t, () -> Algorithm.preShooterMove(300))
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose1))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready,pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(t, () -> Algorithm.preShooterMove(300))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Ready,pickup3Pose))
                .setLinearHeadingInterpolation(pickup3Ready.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(t, () -> Algorithm.preShooterMove(300))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, controlScorePose3, scorePose3))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
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
//                    Algorithm.preShooterMove(500);
                    follower.followPath(grabPickup1, true);

                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    setPathState(50);
                }
                break;

            case 50:
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
                    follower.setMaxPower(lowMaxPower);
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
                    follower.followPath(runto3);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(lowMaxPower);
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
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    @Override
    public void loop() {
        if (follower == null) {
            telemetry.addLine("FOLLOWER IS NULL");
            telemetry.update();
            return;
        }
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
        pathTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

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