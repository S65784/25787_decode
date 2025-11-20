package org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 一个提供机器人位姿和速度信息的通用接口。
 * 坐标系遵循标准的右手笛卡尔坐标系：
 * - Y 轴: 向前为正
 * - X 轴: 向右为正
 * - 航向角 (Heading): 逆时针为正
 */
public interface IPoseProvider {

    /**
     * 更新从硬件读取的最新数据。在每次循环中都应调用此方法。
     */
    void update();

    /**
     * 设置机器人当前的位置
     * @param pose
     */
    void setPose(Pose2D pose);

    /**
     * 初始化硬件
     */
    void initialize();

    /**
     * 获取机器人当前在X轴上的位置。
     * @param unit 距离单位。
     * @return X轴坐标。
     */
    double getX(DistanceUnit unit);

    /**
     * 获取机器人当前在Y轴上的位置。
     * @param unit 距离单位。
     * @return Y轴坐标。
     */
    double getY(DistanceUnit unit);

    /**
     * 获取机器人当前的航向角。
     * @param unit 角度单位。
     * @return 航向角。
     */
    double getHeading(AngleUnit unit);

    /**
     * 获取机器人当前在X轴上的速度。
     * @param unit 距离单位。
     * @return X轴方向的速度。
     */
    double getXVelocity(DistanceUnit unit);

    /**
     * 获取机器人当前在Y轴上的速度。
     * @param unit 距离单位。
     * @return Y轴方向的速度。
     */
    double getYVelocity(DistanceUnit unit);

    /**
     * 获取机器人当前的角速度。
     * @param unit 角度单位。
     * @return 围绕Z轴旋转的角速度。
     */
    double getHeadingVelocity(AngleUnit unit);

    /**
     * 重置位置和IMU。
     */
    void reset();

    /**
     * 获取设备的运行状态。
     * @return 状态字符串。
     */
    String getDeviceStatus();

    /**
     * 获取设备的数据更新频率。
     * @return 频率 (Hz)。
     */
    double getUpdateFrequency();
}