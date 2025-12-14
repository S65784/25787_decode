package org.firstinspires.ftc.teamcode.pedroPathing.pidTTT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// @Config 注解让FTC Dashboard可以识别并修改这个类中的 public static 变量
@Config
@TeleOp(name = "Motor PIDF Tuner (Dashboard)", group = "Tuning")
public class MotorPIDFTunerDashboard extends LinearOpMode {

    // 定义电机对象
    private DcMotorEx motor;

    // FtcDashboard 实例
    private FtcDashboard dashboard;

    // *****************************************************************************************
    //                                  用户需要修改/调试的常量
    // *****************************************************************************************

    // 在这里输入你从电机规格中查到的每转编码器刻度数
    public static final double MOTOR_TICK_COUNT = 28; // 例如, goBILDA 5203系列 19.2:1 电机


    // 你的目标RPM - 现在可以通过Dashboard实时修改！
    public static double TARGET_RPM = 2000;

    // PIDF 系数 - 现在可以通过Dashboard实时修改！
    // P, I, D, F 必须是独立的 public static 变量才能被Dashboard识别
    public static double P = 50, I = 0, D = 10;
    public static double F = 13;


    // *****************************************************************************************

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化 FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        // 使用 MultipleTelemetry 可以同时将数据显示在手机屏幕和Dashboard上
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 从硬件映射中获取电机
        motor = hardwareMap.get(DcMotorEx.class, "Shooter");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("准备开始...");
        telemetry.addLine("连接到FTC Dashboard，并打开浏览器访问 http://192.168.43.1:8080/dash");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                // 从Dashboard实时更新PIDF系数
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                // 将目标RPM转换为每秒的编码器刻度数
                double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;

                // 设置电机的目标速度
                motor.setVelocity(targetTicksPerSecond);

                // 获取当前电机的速度（单位：刻度/秒）
                double currentVelocityTicks = motor.getVelocity();

                // 将当前速度从 刻度/秒 转换为 RPM
                double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;

                // 创建一个数据包，用于发送到Dashboard
                TelemetryPacket packet = new TelemetryPacket();

                // 将你需要绘制图表的数据放入数据包
                // packet.put(key, value)
                packet.put("目标 RPM", TARGET_RPM);
                packet.put("当前 RPM", currentRPM);
                packet.put("误差 (RPM)", TARGET_RPM - currentRPM);

                // 发送数据包到Dashboard
                dashboard.sendTelemetryPacket(packet);

                // 同时在手机屏幕上显示简单数据
                telemetry.addData("目标 RPM", TARGET_RPM);
                telemetry.addData("当前 RPM", "%.2f", currentRPM);
                telemetry.update();
            }
        }

        // 停止电机
        motor.setPower(0);
    }
}