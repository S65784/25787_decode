package org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 使用 goBILDA Pinpoint 设备来实现 IPoseProvider 接口。
 * 这个类封装了所有与 Pinpoint 驱动相关的细节。
 */
public class PinpointPoseProvider implements IPoseProvider {

    private final GoBildaPinpointDriver odo;
    private Pose2D currentPose;

    public PinpointPoseProvider(HardwareMap hardwareMap, String deviceName) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
    }

    /**
     * 初始化Pinpoint设备。在OpMode的init阶段调用。
     */

    // TODO:本地化
    public void initialize() {
        // 根据您的机器人配置进行设置
        odo.setOffsets(0, -60, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        reset(); // 初始重置
    }

    @Override
    public void update() {
        odo.update();
        currentPose = odo.getPosition();
    }

    @Override
    public double getX(DistanceUnit unit) {
        return currentPose != null ? currentPose.getY(unit) : 0;
    }

    @Override
    public double getY(DistanceUnit unit) {
        return currentPose != null ? currentPose.getX(unit) : 0;
    }

    @Override
    public double getHeading(AngleUnit unit) {
        return currentPose != null ? currentPose.getHeading(unit) : 0;
    }

    @Override
    public double getXVelocity(DistanceUnit unit) {
        return odo.getVelY(unit);
    }

    @Override
    public double getYVelocity(DistanceUnit unit) {
        return odo.getVelX(unit);
    }

    @Override
    public double getHeadingVelocity(AngleUnit unit) {
        double headingVelocityInDegrees = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        if (unit == AngleUnit.RADIANS) {
            return Math.toRadians(headingVelocityInDegrees);
        } else {
            return headingVelocityInDegrees;
        }
    }

    @Override
    public void reset() {
        odo.resetPosAndIMU();
    }

    public void recalibrateIMU() {
        odo.recalibrateIMU();
    }

    @Override
    public String getDeviceStatus() {
        return odo.getDeviceStatus().toString();
    }

    @Override
    public double getUpdateFrequency() {
        return odo.getFrequency();
    }

    /**
     * 使用一个标准的场地坐标系Pose2D来强制重置Pinpoint的内部位置。
     * 这个方法会处理所有必要的坐标系转换。
     * @param fieldPose 从AprilTag等外部传感器获取的，以场地标准坐标系定义的Pose2D对象。
     *                  (+Y 向前, +X 向右)
     */
    public void setPose(Pose2D fieldPose) {
        if (fieldPose == null) {
            return;
        }

        // --- 坐标系转换 ---
        // 我们的目标: 当调用 setPose(aprilTagPose) 后，
        // getX() 和 getY() 方法应该返回与 aprilTagPose 一致的值（考虑到您在TeleOp中的符号翻转）。

        // 根据您在TeleOp中的用法:
        // 最终X = -getX()  => 最终X = -currentPose.getY() => 最终X = -odo.yPosition
        // 最终Y = getY()   => 最终Y = currentPose.getX()  => 最终Y = odo.xPosition

        // 因此，为了让最终坐标与AprilTag坐标匹配，我们需要设置:
        // odo.xPosition = aprilTagPose.getY()
        // odo.yPosition = -aprilTagPose.getX()

        // **【已修正】** 使用正确的 Pose2D 构造函数，明确指定单位
        Pose2D poseForDriver = new Pose2D(
                DistanceUnit.MM,                                  // 明确指定距离单位
                fieldPose.getY(DistanceUnit.MM),                  // Pinpoint的X(向前)应设为场地的Y(向前)
                -fieldPose.getX(DistanceUnit.MM),                 // Pinpoint的Y(向左)应设为场地的-X(向右)
                AngleUnit.RADIANS,                                // 明确指定角度单位
                fieldPose.getHeading(AngleUnit.RADIANS)           // 航向角单位一致
        );

        // 调用底层驱动的setPosition方法，将转换后的坐标写入硬件
        odo.setPosition(poseForDriver);

        // 至关重要：在强制设定新位置后，立即调用update()
        // 这会使provider内部的 currentPose 变量刷新为刚刚设定的新值，
        // 从而避免了在同一个循环中读到旧数据的延迟问题。
        update();
    }
}