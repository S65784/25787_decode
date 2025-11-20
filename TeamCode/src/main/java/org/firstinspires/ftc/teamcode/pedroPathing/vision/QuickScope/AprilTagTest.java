package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// 导入所有需要的类
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "AprilTag 定位测试", group = "Tests")
public class AprilTagTest extends LinearOpMode {

    private AprilTagLocalizer aprilTagLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        telemetry.addLine("初始化完成，等待开始 (Press Start)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Pose2D currentPose = aprilTagLocalizer.getRobotPose();

            if (currentPose != null) {
                telemetry.addLine("成功检测到AprilTag!");
                telemetry.addData("坐标 X (cm)", "%.2f", currentPose.getX(DistanceUnit.CM));
                telemetry.addData("坐标 Y (cm)", "%.2f", currentPose.getY(DistanceUnit.CM));
                telemetry.addData("朝向 Heading (度)", "%.2f", currentPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("未检测到AprilTag...");
                telemetry.addLine("请将摄像头对准场上的标签");
            }

            telemetry.update();

            sleep(20);
        }

        aprilTagLocalizer.close();
    }
}