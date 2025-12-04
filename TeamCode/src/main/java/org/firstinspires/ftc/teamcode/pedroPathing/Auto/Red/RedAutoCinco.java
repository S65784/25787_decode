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

@Autonomous(name = "红色近端合作(推gate版)", group = "Competition")
public class RedAutoCinco extends OpMode {
    private Algorithm Algorihthm;
    private TelemetryManager telemetryManager;
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public static double getPointPreX = 93.9;
    public static double getPointX = 126.5;
    public static double Point1Y = 82;
    public static double Point2Y = 58;



    // Define Poses
    private final Pose startPose = new Pose(123.5, 122.5, Math.toRadians(38));

    private final Pose scorePose = new Pose(112.52, 115.52, Math.toRadians(38));
    private final Pose controlScorePose1 = new Pose(104.91220028208745, 77.99153737658675, Math.toRadians(32));
    private final Pose scorePose1 = new Pose(113, 111, Math.toRadians(32));
    private final Pose controlScorePose2 = new Pose(104, 60.3, Math.toRadians(32));
    private final Pose scorePose2 = new Pose(113, 109, Math.toRadians(32));



    private final Pose controlPickup1Ready = new Pose(93.94464033850494, 96.27080394922427, Math.toRadians(0));
    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(getPointX, Point1Y, Math.toRadians(0));

    private final Pose controlTheGate = new Pose(114.05183356840621, 69.86741889985896, Math.toRadians(0));
    private final Pose theGate = new Pose(128.26, 70.6798307475317468, Math.toRadians(0));

    private final Pose controlPickup2Ready = new Pose(76.6, 61.9, Math.toRadians(0));
    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(getPointX, Point2Y, Math.toRadians(0));


    private final Pose end = new Pose(120.78023407022106, 93.6, Math.toRadians(32));

    private Path scorePreload, runto1, runto2;
    private PathChain runTheGate, grabPickup1, grabPickup2, scorePickup1, scorePickup2,endpath;

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
                .build();
        runTheGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose,controlTheGate, theGate))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), theGate.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(theGate, controlScorePose1, scorePose1))
                .setLinearHeadingInterpolation(theGate.getHeading(), scorePose1.getHeading())
                .build();



        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Ready,pickup2Pose))
                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();



        endpath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose2, end))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), end.getHeading())
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
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
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

                    Algorithm.draw();
                    follower.setMaxPower(0.37);
                    follower.followPath(grabPickup1, true);
                    setPathState(30);
                }
                break;


            case 30:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    Algorithm.stopShoot();

                    Algorithm.sleep(7000);

                    follower.followPath(runTheGate, true);
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;



            case 5:
                if (!follower.isBusy()) {
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
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
                    Algorithm.draw();
                    follower.setMaxPower(0.37);
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
                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
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

//        telemetryManager.addData("test",Algorithm.test);
//        telemetryManager.update(telemetry);
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