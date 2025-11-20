package org.firstinspires.ftc.teamcode.pedroPathing.Tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.VisionAlgorithm;

@TeleOp
public class DOS extends LinearOpMode {
    private Algorithm Algorihthm;


    // ... (其他变量声明保持不变)
    private ElapsedTime runtime = new ElapsedTime();

    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;


    @Override
    public void runOpMode() {

        // 1. 初始化所有硬件 (与之前相同)
        Algorihthm = new Algorithm(hardwareMap);

        // 2. 初始化IMU (不再重置Yaw)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        Algorithm.imu.initialize(new IMU.Parameters(orientationOnRobot));
        // --- 修改: 不再调用 imu.resetYaw() ---
        // imu.resetYaw(); // 注释掉或删除此行

        // 3. 赛前AprilTag校准


        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = VisionAlgorithm.aprilTagLocalizer.getRobotPose();
            if (initialPose != null) {
                telemetry.addLine(">> 已成功检测到AprilTag! <<");
                telemetry.addData("检测到的初始位置", VisionAlgorithm.formatPose(initialPose));
                telemetry.addLine("松开手柄，随时可以按Start开始...");
            } else {
                telemetry.addLine("正在寻找AprilTag...");
                telemetry.addLine("请将摄像头对准场上的AprilTag");
            }
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) return;

        // 4. 开始比赛，进行最后一次校准，并设置IMU偏移量
        telemetry.addLine("正在进行最后一次AprilTag初始校准...");
        telemetry.update();

        VisionAlgorithm.someMethodForVision();

        VisionAlgorithm.aprilTagLocalizer.close();
        VisionAlgorithm.previousTime = System.nanoTime();
        runtime.reset();

        // 5. 主循环 (大部分保持不变)
        String targetAlliance = "Red";

        while (opModeIsActive()) {

            // --- A. 获取机器人实时状态 ---
            VisionAlgorithm.pinpointPoseProvider.update();
            // ... (状态获取代码不变)

            if (VisionAlgorithm.direction_deg < 0) VisionAlgorithm.direction_deg += 360;

            double distance = Math.hypot(-VisionAlgorithm.pinpointPoseProvider.getX(DistanceUnit.CM),
                    VisionAlgorithm.pinpointPoseProvider.getY(DistanceUnit.CM));

            TARGET_RPM = 3600 - distance * 4;


            VisionAlgorithm.autoTurnControl(Algorithm.flag(gamepad1.right_bumper));

            if (gamepad1.left_bumper) {
                VisionAlgorithm.pinpointPoseProvider.reset();
                // --- 修改: 手动重置时，也重置IMU的场地0度参考点 ---
                Algorithm.imu.resetYaw();
                VisionAlgorithm.headingOffset = 0; // 重置后，当前方向就是新的0度，所以偏移量为0
                telemetry.addLine("里程计和IMU已手动重置!");
            }
        }
    }
    /**
     * 获取当前机器人基于场地的航向（偏航角）
     * @return 校准后的场地航向角度（-180 到 180度）
     */

}