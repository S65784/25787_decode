package org.firstinspires.ftc.teamcode.pedroPathing.Auto.Red;

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

@Autonomous(name = "红色离开", group = "Competition")
public class RedAutoCuatro extends OpMode {
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
    private final Pose startPose = new Pose(85, 8.4, Math.toRadians(0));
    private final Pose leavePose = new Pose(109, 8.4, Math.toRadians(0));



    private Path leave;

    public void buildPaths() {
//ichi ni san shi/yon go roku shichi/nana hachi kyu jyu
        leave = new Path(new BezierLine(startPose, leavePose));
        leave.setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(leave);
                setPathState(-1);
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


