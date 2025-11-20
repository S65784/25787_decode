package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * [坐标系转换版] 封装了AprilTag定位功能的独立类。
 * 此版本包含了将AprilTag默认的“中心原点”坐标系转换为我们期望的“左下角原点”坐标系的关键逻辑。
 * [新增功能] 同时增加了检测特定AprilTag ID (21, 22, 23) 以判断位置的功能。
 */
public class AprilTagLocalizer {
    //TODO: 摄像机姿态本地化
    private static final Position cameraPosition = new Position(DistanceUnit.CM,
            -9.7, 6.8, 38.5, 0);
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -56, -90, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double HALF_FIELD_MM = 72 * 25.4;

    // ********** 新增内容开始 **********
    // 为需要特殊检测的AprilTag ID定义常量，提高代码可读性和可维护性。
    // 假设这些ID对应 Spike Mark 的三个位置：左、中、右。
    private static final int SPIKE_MARK_ID_LEFT = 21;   // 返回 1
    private static final int SPIKE_MARK_ID_MIDDLE = 22; // 返回 2
    private static final int SPIKE_MARK_ID_RIGHT = 23;  // 返回 3
    // ********** 新增内容结束 **********


    public AprilTagLocalizer(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setNumThreads(4)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(1);
        //TODO: 修改摄像头初始化
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * 核心方法：获取当前机器人经过坐标系转换后的、正确的Pose2D。
     * @return 如果看到任何有效的AprilTag，则返回一个精确的Pose2D；否则返回null。
     */
    public Pose2D getRobotPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // 这个方法的逻辑完全保持不变
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.robotPose != null) {
                double aprilTagX = detection.robotPose.getPosition().x;
                double aprilTagY = detection.robotPose.getPosition().y;
                double aprilTagYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                double finalX_mm = aprilTagY + HALF_FIELD_MM;
                double finalY_mm = -aprilTagX + HALF_FIELD_MM;
                double finalHeading_deg = aprilTagYaw - 90;

                return new Pose2D(
                        DistanceUnit.MM,
                        finalX_mm,
                        finalY_mm,
                        AngleUnit.DEGREES,
                        finalHeading_deg
                );
            }
        }

        return null;
    }

    // ********** 新增方法开始 **********
    /**
     * 新增功能：检测特定的Spike Mark AprilTag。
     * 这个方法专门用于在自动阶段开始时，判断随机化的位置。
     * 它会持续检查，只要看到ID为21、22或23的标签，就立刻返回对应的值。
     *
     * @return 1 - 如果检测到ID为 21 的AprilTag。
     *         2 - 如果检测到ID为 22 的AprilTag。
     *         3 - 如果检测到ID为 23 的AprilTag。
     *         0 - 如果当前视野内没有检测到以上任何一个AprilTag。
     */
    public int getSpikeMarkLocation() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // 遍历当前帧检测到的所有AprilTag
        for (AprilTagDetection detection : currentDetections) {
            // 确保检测结果是有效的
            if (detection.metadata != null) {
                // 使用 switch 语句检查ID，更清晰高效
                switch (detection.id) {
                    case SPIKE_MARK_ID_LEFT:
                        return 1; // 看到ID 21，立即返回 1
                    case SPIKE_MARK_ID_MIDDLE:
                        return 2; // 看到ID 22，立即返回 2
                    case SPIKE_MARK_ID_RIGHT:
                        return 3; // 看到ID 23，立即返回 3
                }
            }
        }

        // 如果循环结束了还没有返回，说明没有看到任何一个目标ID
        return 0;
    }
    // ********** 新增方法结束 **********


    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}