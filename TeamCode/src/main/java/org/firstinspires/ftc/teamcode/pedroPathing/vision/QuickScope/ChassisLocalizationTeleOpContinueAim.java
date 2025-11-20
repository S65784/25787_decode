package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope; // 使用您指定的包名

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.PinpointPoseProvider;

import java.util.Locale;

@TeleOp(name = "Archer 自动瞄准系统 (V8.4 - 持续瞄准)", group = "Main") // 修改: 版本号和功能描述更新
public class ChassisLocalizationTeleOpContinueAim extends LinearOpMode {
    // 定位与瞄准系统
    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;

    // 发射与伺服机构
    private Servo LeftPitch;
    private Servo RightPitch;
    private Servo RightBoard;
    private Servo LeftBoard;
    private DcMotorEx LeftShooter;
    private DcMotorEx RightShooter;
    private DcMotorEx InhaleMotor;
    private DcMotorEx IntakeMotor;
    public static final double SHOOTER_TICKS_PER_REV = 28;
    public static final double SHOOTER_GEAR_RATIO = 1.0;

    // --- 底盘与IMU硬件 ---
    private IMU imu;
    private DcMotor LeftFrontMotor;
    private DcMotor LeftBehindMotor;
    private DcMotor RightFrontMotor;
    private DcMotor RightBehindMotor;

    // --- PID转向控制参数 ---
    private double targetHeading = 0;
    private boolean isAimingContinuously = false; // 修改: 变量名更能反映其功能
    private final double TURN_POWER = 1.0; // 自动转向时的最大马力
    private final double HEADING_THRESHOLD = 0.5; // 目标角度容忍误差 (度)
    private final double P_TURN_GAIN = 0.1; // 比例增益
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.00001; // 积分增益
    private final double D_GAIN = 0.006;  // 微分增益

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化设备
        initializeVisionAndShooter();
        initializeChassisAndIMU();

        telemetry.addLine("初始化完成，请将机器人对准AprilTag");
        telemetry.update();

        // 2. 等待开始 (AprilTag校准)
        waitForStartWithAprilTag();
        if (isStopRequested()) return;

        // 3. 最终校准
        performInitialLocalization();

        // 4. 关闭AprilTag以节省资源
        aprilTagLocalizer.close();

        // 5. 主循环
        String targetAlliance = "Red";
        LaunchSolution savedSolution = null;

        // 初始化PID时间戳
        previousTime = System.nanoTime();

        while (opModeIsActive()) {
            // --- A. 获取机器人实时状态 (输入) ---
            pinpointPoseProvider.update();
            double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
            double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);
            double normalizationX = robotX_cm / 365.76;
            double normalizationY = robotY_cm / 365.76;
            double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
            double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
            double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
            double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
            double direction_deg = Math.toDegrees(direction_rad);
            if (direction_deg < 0) direction_deg += 360;

            // --- B. 获取手柄输入 ---
            if (gamepad1.x) targetAlliance = "Blue";
            if (gamepad1.b) targetAlliance = "Red";
            if (gamepad1.a) pinpointPoseProvider.reset();
            if (gamepad1.left_bumper) {
                imu.resetYaw();
                telemetry.addLine(">> IMU偏航角已重置 <<");
            }

            // =================================================================================
            // --- 核心修改: 从 "按一下" 变为 "按住" ---
            // =================================================================================
            if (gamepad1.right_bumper) {
                // 只要按住Right Bumper，就持续进行解算
                CalculationParams currentParams = new CalculationParams(
                        normalizationX, normalizationY, speed_m_s, direction_deg, targetAlliance);
                savedSolution = archerLogic.calculateSolution(currentParams);

                if (savedSolution != null) {
                    // 如果解算成功，实时更新目标角度，并设置瞄准状态为true
                    targetHeading = savedSolution.aimAzimuthDeg;
                    isAimingContinuously = true;
                } else {
                    // 如果在按住期间丢失目标，则停止自动瞄准，恢复手动
                    isAimingContinuously = false;
                }
            } else {
                // 如果松开Right Bumper，立即停止自动瞄准
                isAimingContinuously = false;
            }

            // --- E. 底盘运动控制 ---
            if (isAimingContinuously) {
                // 当处于持续瞄准状态时，执行自动转向
                // 注意: performAutoTurn内部仍然允许左摇杆控制平移
                performAutoTurn();
            } else {
                // 否则，完全手动驾驶
                manualDrive();
            }

            // --- F. 遥测数据显示与硬件控制 (始终使用 'savedSolution') ---
            displayTelemetry(targetAlliance, savedSolution, normalizationX, normalizationY, speed_m_s, direction_deg);
            applySolutionToHardware(savedSolution);

            sleep(20);
        }

        // OpMode结束时确保所有电机停止
        stopMotors();
        LeftShooter.setVelocity(0);
        RightShooter.setVelocity(0);
    }

    // ---------------------------------------------------------------------------------------------
    // 初始化方法 (无变化)
    // ---------------------------------------------------------------------------------------------
    private void initializeVisionAndShooter() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        archerLogic = new ArcherLogic();

        RightPitch = hardwareMap.get(Servo.class, "RightPitch");
        LeftPitch = hardwareMap.get(Servo.class, "LeftPitch");
        RightBoard = hardwareMap.get(Servo.class, "RightBoard");
        LeftBoard = hardwareMap.get(Servo.class, "LeftBoard");
        pinpointPoseProvider.initialize();

        LeftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotorEx.class, "RightShooter");
        InhaleMotor = hardwareMap.get(DcMotorEx.class, "InhaleMotor");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        LeftShooter.setDirection(DcMotor.Direction.REVERSE);
        RightShooter.setDirection(DcMotor.Direction.REVERSE);
        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftShooter.setVelocityPIDFCoefficients(160, 0, 90, 18);
        RightShooter.setVelocityPIDFCoefficients(170, 0, 20, 15.85);
    }

    private void initializeChassisAndIMU() {
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        LeftBehindMotor = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        RightBehindMotor = hardwareMap.get(DcMotor.class, "RightBehindMotor");

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftBehindMotor.setDirection(DcMotor.Direction.FORWARD);
        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        RightBehindMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void waitForStartWithAprilTag() {
        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = aprilTagLocalizer.getRobotPose();
            if (initialPose != null) {
                telemetry.addLine(">> 已成功检测到AprilTag! <<");
                telemetry.addData("检测到的初始位置", formatPose(initialPose));
                telemetry.addLine("松开手柄，随时可以按Start开始...");
            } else {
                telemetry.addLine("正在寻找AprilTag...");
            }
            telemetry.update();
            sleep(20);
        }
    }

    private void performInitialLocalization() {
        telemetry.addLine("正在进行最后一次AprilTag初始校准...");
        telemetry.update();
        Pose2D aprilTagInitialPose = aprilTagLocalizer.getRobotPose();
        if (aprilTagInitialPose != null) {
            pinpointPoseProvider.setPose(aprilTagInitialPose);
            telemetry.addLine(">> 校准成功! AprilTag坐标已设置为里程计初始值 <<");
        } else {
            telemetry.addLine(">> 未找到AprilTag! 里程计将从(0,0,0)开始 <<");
        }
        telemetry.update();
        sleep(1000);
    }

    // ---------------------------------------------------------------------------------------------
    // 主循环内的方法 (修改遥测部分)
    // ---------------------------------------------------------------------------------------------
    private void manualDrive() {
        double axial   = -gamepad1.left_stick_y;  // 前后
        double lateral =  gamepad1.left_stick_x;  // 平移
        double yaw     =  gamepad1.right_stick_x; // 旋转

        setMecanumPower(axial, lateral, yaw);
    }

    private void applySolutionToHardware(LaunchSolution solution) {
        if (solution != null) {
            double finalpitch = 90 - solution.launcherAngle;
            double leftservoposition = finalpitch * 0.028571428571428 - 0.05;
            double rightservoposition = 1 - leftservoposition - 0.007;

            LeftBoard.setPosition(0.82);
            RightBoard.setPosition(0.618);
            LeftPitch.setPosition(leftservoposition);
            RightPitch.setPosition(rightservoposition);

            // 以下部分可以根据需要取消注释
            // double targetRpm = solution.motorRpm;
            // double ticksPerSecond = rpmToTicksPerSecond(targetRpm);
            // LeftShooter.setVelocity(ticksPerSecond);
            // RightShooter.setVelocity(ticksPerSecond);
            // IntakeMotor.setPower(1);
            // InhaleMotor.setPower(0.55);
        }
    }

    private void displayTelemetry(String targetAlliance, LaunchSolution solution, double normX, double normY, double speed, double dir) {
        telemetry.addLine("--- Archer自动瞄准系统 (V8.4) ---");
        telemetry.addData("当前目标", "%s Alliance (按X/B切换)", targetAlliance);
        // --- 修改: 更新用户提示 ---
        telemetry.addLine("\n>> 按住 RIGHT BUMPER 持续瞄准 <<");
        telemetry.addLine(">> 按 LEFT BUMPER 重置IMU朝向 <<");

        if (solution != null) {
            telemetry.addLine("\n--- 已锁定的解算方案 ---");
            telemetry.addData("发射电机转速 (RPM)", "%.0f", solution.motorRpm);
            telemetry.addData("目标偏航角 (Yaw)", "%.2f deg", solution.aimAzimuthDeg);
            telemetry.addData("俯仰角 (Pitch)", "%.2f deg", 90 - solution.launcherAngle);
            double ticksPerSecond = rpmToTicksPerSecond(solution.motorRpm);
            telemetry.addData("电机目标速度 (Ticks/Sec)", "%.2f", ticksPerSecond);
        } else {
            telemetry.addLine("\n--- 等待第一次解算 ---");
        }

        telemetry.addLine("\n--- 底盘与转向 ---");
        telemetry.addData("当前航向", "%.1f°", getHeading());
        telemetry.addData("目标航向", "%.1f°", targetHeading);
        // --- 修改: 更新状态显示 ---
        telemetry.addData("自动瞄准状态", isAimingContinuously ? "持续瞄准中..." : "手动控制");


        telemetry.addLine("\n--- 机器人实时状态 ---");
        telemetry.addData("归一化坐标", String.format(Locale.US, "{X: %.3f, Y: %.3f}", normX, normY));
        telemetry.addData("实时速度", String.format(Locale.US,"{Spd: %.2f m/s, Dir: %.1f deg}", speed, dir));
        telemetry.update();
    }


    // ---------------------------------------------------------------------------------------------
    // PID 转向与底盘控制 (无变化)
    // ---------------------------------------------------------------------------------------------

    private void performAutoTurn() {
        double headingError = calculateHeadingError(getHeading(), targetHeading);

        // --- 修改: 移除自动停止逻辑，因为只要按住就会持续修正 ---
        // if (Math.abs(headingError) <= HEADING_THRESHOLD) {
        //     stopMotors();
        //     isTurningToTarget = false; // 旧的状态变量
        //     return;
        // }

        // 只要在瞄准模式，就持续计算PID输出
        double turnPower = calculatePIDOutput(headingError);

        // 在自动转向时，我们只控制旋转，但允许手动平移
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        setMecanumPower(axial, lateral, turnPower);
    }

    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9;
        if (deltaTime == 0) deltaTime = 1e-9;

        double proportional = P_TURN_GAIN * error;
        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / I_GAIN;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;
        double derivative = D_GAIN * (error - previousError) / deltaTime;
        double output = proportional + integral + derivative;

        previousError = error;
        previousTime = currentTime;

        return Range.clip(output, -TURN_POWER, TURN_POWER);
    }

    // --- 修改: 不再需要 resetPID，因为PID会根据持续变化的误差自然调整 ---
    // private void resetPID() { ... }

    private void setMecanumPower(double drive, double strafe, double turn) {
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        LeftFrontMotor.setPower(leftFrontPower);
        RightFrontMotor.setPower(rightFrontPower);
        LeftBehindMotor.setPower(leftBackPower);
        RightBehindMotor.setPower(rightBackPower);
    }

    private void stopMotors() {
        setMecanumPower(0, 0, 0);
    }

    private double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = currentHeading - targetHeading;
        while (error > 180)  error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // ---------------------------------------------------------------------------------------------
    // 辅助工具方法 (无变化)
    // ---------------------------------------------------------------------------------------------
    private String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO) / 60.0;
    }
}