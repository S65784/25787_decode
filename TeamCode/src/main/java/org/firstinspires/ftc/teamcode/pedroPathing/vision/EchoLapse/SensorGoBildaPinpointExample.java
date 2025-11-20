package org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="使用接口的goBILDA Pinpoint示例", group="Linear OpMode")
public class SensorGoBildaPinpointExample extends LinearOpMode {

    private IPoseProvider poseProvider;
    private double oldTime = 0;

    @Override
    public void runOpMode() {
        // TODO:修改初始化信息
        PinpointPoseProvider pinpoint = new PinpointPoseProvider(hardwareMap, "pinpoint");
        pinpoint.initialize();
        poseProvider = pinpoint;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("说明", "坐标系已设置为标准笛卡尔坐标系");
        telemetry.addData("说明", "Y轴: 前方为正, X轴: 右侧为正");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            // 2. 在循环开始时更新数据
            poseProvider.update();

            // 3. 使用接口中的方法获取位移和速度
            // 现在外部程序可以非常轻松地通过调用这些方法来获取所需数据
            double cartesianX_mm = -poseProvider.getX(DistanceUnit.MM);
            double cartesianY_mm = poseProvider.getY(DistanceUnit.MM);
            double cartesianHeading_deg = poseProvider.getHeading(AngleUnit.DEGREES);

            double cartesianVelX_mm_s = -poseProvider.getXVelocity(DistanceUnit.MM);
            double cartesianVelY_mm_s = poseProvider.getYVelocity(DistanceUnit.MM);
            double headingVel_deg_s = poseProvider.getHeadingVelocity(AngleUnit.DEGREES);

            // 处理手柄输入
            if (gamepad1.a){
                poseProvider.reset();
            }
            if (gamepad1.b && poseProvider instanceof PinpointPoseProvider){
                // 如果需要调用Pinpoint特有的方法，可以进行类型转换
                ((PinpointPoseProvider) poseProvider).recalibrateIMU();
            }

            // --- 以下是遥测数据显示，与核心逻辑分离 ---

            String positionData = String.format(Locale.US, "{X: %.2f mm, Y: %.2f mm, H: %.2f deg}",
                    cartesianX_mm,
                    cartesianY_mm,
                    cartesianHeading_deg
            );
            telemetry.addData("笛卡尔坐标 (位置)", positionData);


            String velocityData = String.format(Locale.US,"{XVel: %.2f mm/s, YVel: %.2f mm/s, HVel: %.2f deg/s}",
                    cartesianVelX_mm_s,
                    cartesianVelY_mm_s,
                    headingVel_deg_s
            );
            telemetry.addData("笛卡尔坐标 (速度)", velocityData);


            telemetry.addData("状态", poseProvider.getDeviceStatus());
            telemetry.addData("Pinpoint 频率", String.format(Locale.US, "%.2f Hz", poseProvider.getUpdateFrequency()));

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = (loopTime > 0) ? 1.0 / loopTime : 0;
            oldTime = newTime;
            telemetry.addData("REV Hub 频率: ", String.format(Locale.US, "%.2f Hz", frequency));
            telemetry.update();
        }
    }
}