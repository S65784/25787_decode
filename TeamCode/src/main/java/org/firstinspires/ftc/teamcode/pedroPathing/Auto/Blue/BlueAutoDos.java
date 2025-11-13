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

@Autonomous(name = "BlueAutoDos(coop--3+'2')", group = "Competition")
public class BlueAutoDos extends OpMode {
    private Algorithm Algorihthm;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static double getPointPreX = 45;
    public static double getPointX = 12;
    public static double Point1Y = 82;
    public static double Point2Y = 58;
    public static double Point3Y = 35;


    // Define Poses
    private final Pose startPose = new Pose(16.5, 122.5, Math.toRadians(142));

    private final Pose scorePose = new Pose(30, 115.9, Math.toRadians(142));
    private final Pose scorePose1 = new Pose(30, 110, Math.toRadians(148));
    private final Pose controlScorePose2 = new Pose(36, 60.3, Math.toRadians(148));
    private final Pose scorePose2 = new Pose(27, 109, Math.toRadians(148));
    private final Pose controlScorePose3 = new Pose(27, 39, Math.toRadians(148));
    private final Pose scorePose3 = new Pose(29, 106, Math.toRadians(148));


    private final Pose controlPickup1Ready = new Pose(48, 97, Math.toRadians(180));
    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(getPointX, Point1Y, Math.toRadians(180));

    private final Pose controlPickup2Ready = new Pose(63.4, 61.9, Math.toRadians(180));
    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(getPointX, Point2Y, Math.toRadians(180));

    private final Pose controlPickup3Ready = new Pose(61, 70, Math.toRadians(180));
    private final Pose pickup3Ready = new Pose(getPointPreX, Point3Y, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(9, Point3Y, Math.toRadians(180));

    private Path scorePreload, runto1, runto2, runto3;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

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
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose1))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();


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
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2700);
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
                    follower.setMaxPower(0.37);
                    Algorithm.draw();
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2700);
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
                    follower.setMaxPower(0.37);
                    Algorithm.draw();
                    follower.followPath(grabPickup2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2700);
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
                    follower.setMaxPower(0.37);
                    Algorithm.draw();
                    follower.followPath(grabPickup3, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 3000);
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