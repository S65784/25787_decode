//package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Blue;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.paths.Path;
//
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
//@Autonomous(name = "侧板27570coop-蓝", group = "Competition")
//public class BlueAutoSeis  extends OpMode {
//    private Algorithm Algorihthm;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    public static double getPointPreX = 45;
//    public static double getPointX = 12;
//    public static double Point1Y = 82;
//    public static double Point2Y = 58;
//
//
//    // Define Poses
//    private final Pose startPose = new Pose(16.5, 122.5, Math.toRadians(142));
//
//    private final Pose scorePose = new Pose(26.4, 115.9, Math.toRadians(142));
//
//    private final Pose controlScorePose1 = new Pose(140-104.91220028208745, 77.99153737658675, Math.toRadians(180-32));
//    private final Pose scorePose1 = new Pose(28, 110, Math.toRadians(148));
//    private final Pose controlScorePose2 = new Pose(36, 60.3, Math.toRadians(148));
//    private final Pose scorePose2 = new Pose(27, 109, Math.toRadians(148));
//
//
//    private final Pose controlPickup1Ready = new Pose(48, 97, Math.toRadians(180));
//    private final Pose pickup1Ready = new Pose(getPointPreX, Point1Y, Math.toRadians(180));
//    private final Pose pickup1Pose = new Pose(getPointX, Point1Y, Math.toRadians(180));
//
//    private final Pose controlTheGate1 = new Pose(59.06086956521739, 82.85217391304347, Math.toRadians(10));
//    private final Pose theGate1 = new Pose(33.81847826086956, 76.17391304347827, Math.toRadians(10));
//    //private final Pose controlTheGate2 = new Pose(140-123.39456981664316, 75.14809590973204, Math.toRadians(180));
//    private final Pose theGate2 = new Pose(13.7, 73.6695652173913, Math.toRadians(0));
//
////    private final Pose controlTheGate1 = new Pose(34.22608695652174, 82.01739130434783, Math.toRadians(0));
////    private final Pose theGate1 = new Pose(14.4, 73.66956521739131, Math.toRadians(0));
//
//    private final Pose controlPickup2Ready = new Pose(63.4, 61.9, Math.toRadians(180));
//    private final Pose pickup2Ready = new Pose(getPointPreX, Point2Y, Math.toRadians(180));
//    private final Pose pickup2Pose = new Pose(getPointX, Point2Y, Math.toRadians(180));
//
//
//    private final Pose end = new Pose(140-120.78023407022106, 93.6, Math.toRadians(32));
//    private Path scorePreload, runto1, runto2;
//    private PathChain runTheGate, grabPickup1, grabPickup2, scorePickup1, scorePickup2, endpath;
//
//    public void buildPaths() {
//
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//
//
//        runto1 = new Path(new BezierCurve(scorePose,controlPickup1Ready,pickup1Ready));
//        runto1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Ready.getHeading());
//
//        runto2 = new Path(new BezierCurve(scorePose1,controlPickup2Ready,pickup2Ready));
//        runto2.setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Ready.getHeading());
//
//
//
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Ready,pickup1Pose))
//                .setLinearHeadingInterpolation(pickup1Ready.getHeading(), pickup1Pose.getHeading())
//                .build();
//        runTheGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup1Pose,controlTheGate1, theGate1))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), theGate1.getHeading())
//                .addPath(new BezierLine(theGate1, theGate2))
//                .setLinearHeadingInterpolation(theGate1.getHeading(), theGate2.getHeading())
//                .build();
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(theGate1, controlScorePose1, scorePose))
//                .setLinearHeadingInterpolation(theGate1.getHeading(), scorePose.getHeading())
//                .build();
//
//
//
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Ready,pickup2Pose))
//                .setLinearHeadingInterpolation(pickup2Ready.getHeading(), pickup2Pose.getHeading())
//                .build();
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pose, controlScorePose2, scorePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//
//
//        endpath = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose2, end))
//                .setLinearHeadingInterpolation(scorePose2.getHeading(), end.getHeading())
//                .build();
//
////        park = new Path(new BezierCurve(new Point(scorePose),
////                new Point(parkControlPose),
////                new Point(parkPose)));
////        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//
//            case 0:
//                follower.followPath(scorePreload);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
//                    setPathState(2);
//                }
//                break;
//
//
//
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(runto1);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//
//                    Algorithm.draw();
//                    follower.setMaxPower(0.37);
//                    follower.followPath(grabPickup1, true);
//                    Algorithm.sleep(7000);
//                    setPathState(30);
//                }
//                break;
//
//
//            case 30:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    Algorithm.stopShoot();
//
//
//
//                    follower.followPath(runTheGate, true);
//                    setPathState(4);
//                }
//                break;
//
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePickup1, true);
//                    setPathState(5);
//                }
//                break;
//
//
//
//            case 5:
//                if (!follower.isBusy()) {
//                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
//                    setPathState(6);
//                }
//                break;
//
//
//
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(runto2);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    Algorithm.draw();
//                    follower.setMaxPower(0.37);
//                    follower.followPath(grabPickup2, true);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    Algorithm.stopShoot();
//                    follower.followPath(scorePickup2, true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                if (!follower.isBusy()) {
//                    Algorithm.shootTime(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, true, 2600);
//                    setPathState(10);
//                }
//                break;
//
//
//
//            case 10:
//                if (!follower.isBusy()) {
//                    follower.followPath(endpath, true);
//                    setPathState(-1);
//                }
//                break;
//
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Status", "Running");
//        telemetry.addData("目标 RPM", Algorithm.targetRPM);
//        telemetry.addData("当前 RPM", "%.2f", Algorithm.getCurrentRPM());
//        telemetry.addData("test",Algorithm.test);
//
//
//        telemetry.update();
//        telemetry.update();
//    }
//
//    @Override
//    public void init() {
//        Algorihthm = new Algorithm(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//        pathTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        setPathState(0);
//
//        Algorithm.servoControl();
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//    }
//
//
//}
