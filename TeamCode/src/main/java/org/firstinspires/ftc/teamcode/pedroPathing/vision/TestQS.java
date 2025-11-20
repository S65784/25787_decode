package org.firstinspires.ftc.teamcode.pedroPathing.vision;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.*;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.*;

import java.util.Locale;

@TeleOp
public class TestQS extends LinearOpMode {

    // ... (其他变量声明保持不变)
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;
    public static final double SHOOTER_TICKS_PER_REV = 28;
    public static final double SHOOTER_GEAR_RATIO = 1.0;
    private double targetHeading = 0;
    private final double TURN_POWER = 1.0;
    private final double HEADING_THRESHOLD = 1.5;
    private final double P_TURN_GAIN = 0.1;
    private final double I_GAIN = 0.0001;
    private final double D_GAIN = 0.003;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private boolean isAutoAiming = false;
    private boolean rightBumperLastState = false;

    // --- 新增: IMU航向角校准偏移量 ---
    private double headingOffset = 0.0;

;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotor blender = null;
    private DcMotorEx shooter = null;

    private Servo block = null;
    private Servo ls = null;
    private Servo rs = null;

    private boolean lastYState = false;
    private boolean state = false;
    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 135, I = 0, D = 80, F = 14;//p=140
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    @Override
    public void runOpMode() {

        // 1. 初始化所有硬件 (与之前相同)
        initializeHardware();

        // 2. 初始化IMU (不再重置Yaw)
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // --- 修改: 不再调用 imu.resetYaw() ---
        // imu.resetYaw(); // 注释掉或删除此行

        // 3. 赛前AprilTag校准
        telemetry.addLine("初始化完成，请将机器人对准AprilTag");
        telemetry.addData("模式", "仅在开始时进行一次AprilTag校准");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            Pose2D initialPose = aprilTagLocalizer.getRobotPose();
            if (initialPose != null) {
                telemetry.addLine(">> 已成功检测到AprilTag! <<");
                telemetry.addData("检测到的初始位置", formatPose(initialPose));
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

        Pose2D aprilTagInitialPose = aprilTagLocalizer.getRobotPose();
        if (aprilTagInitialPose != null) {
            pinpointPoseProvider.setPose(aprilTagInitialPose);

            // --- 核心修改: 计算并设置IMU偏移量 ---
            double aprilTagHeading = aprilTagInitialPose.getHeading(AngleUnit.DEGREES);
            double rawImuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingOffset = aprilTagHeading - rawImuHeading;

            telemetry.addLine(">> 校准成功! AprilTag坐标已设置为里程计初始值 <<");
            telemetry.addData("初始坐标", formatPose(aprilTagInitialPose));
            telemetry.addData("IMU已校准到场地坐标", "偏移量: %.2f deg", headingOffset);

        } else {
            // 如果未找到AprilTag, 则将当前朝向作为0度 (与之前的resetYaw效果相同)
            telemetry.addLine(">> 未找到AprilTag! 里程计将从(0,0,0)开始 <<");
            telemetry.addLine("IMU将使用当前朝向作为0度");
            imu.resetYaw(); // 仅在失败时重置，以保证有一个已知的起点
            headingOffset = 0; // 偏移量为0
        }
        telemetry.update();
        sleep(2000); // 增加延时，让用户能看到校准结果

        aprilTagLocalizer.close();
        previousTime = System.nanoTime();
        runtime.reset();

        // 5. 主循环 (大部分保持不变)
        String targetAlliance = "Red";

        while (opModeIsActive()) {

            // --- A. 获取机器人实时状态 ---
            pinpointPoseProvider.update();
            // ... (状态获取代码不变)
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

            double currentVelocityTicks = shooter.getVelocity();
            double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;

            double distence = Math.hypot(-pinpointPoseProvider.getX(DistanceUnit.CM),
                    pinpointPoseProvider.getY(DistanceUnit.CM));

            TARGET_RPM = 3600 - distence*4 ;

            boolean volocitycheck = false;

            if((TARGET_RPM + ErrorRange)>currentRPM && currentRPM>(TARGET_RPM - ErrorRange)) {
                volocitycheck = true;
            }
            else {
                volocitycheck = false;
            }


            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            shooter.setVelocity(targetTicksPerSecond);
            // --- B. 获取手柄输入 ---
            if (gamepad1.dpad_left) targetAlliance = "Blue";
            if (gamepad1.dpad_right) targetAlliance = "Red";
            if (gamepad1.left_bumper) {
                pinpointPoseProvider.reset();
                // --- 修改: 手动重置时，也重置IMU的场地0度参考点 ---
                imu.resetYaw();
                headingOffset = 0; // 重置后，当前方向就是新的0度，所以偏移量为0
                telemetry.addLine("里程计和IMU已手动重置!");
            }

            // --- C. Toggle自动对准模式 (不变) ---
            boolean rightBumperCurrentState = gamepad1.right_bumper;
            if (rightBumperCurrentState && !rightBumperLastState) {
                isAutoAiming = !isAutoAiming;
                if (isAutoAiming) {
                    resetPID();
                } else {
                }
            }
            rightBumperLastState = rightBumperCurrentState;

            // --- D & E. 计算瞄准方案 & 控制发射机构 (不变) ---
            CalculationParams currentParams = new CalculationParams(normalizationX, normalizationY, speed_m_s, direction_deg, targetAlliance);
            LaunchSolution solution = archerLogic.calculateSolution(currentParams);

            // --- F. 底盘控制逻辑 (不变) ---
            if (isAutoAiming) {
                if (solution != null) {
                    targetHeading = solution.aimAzimuthDeg;
                    performAutoTurn();

                } else {
                }
            } else {
                double max;

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double axial   = -gamepad1.left_stick_y;
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                double newAxial = axial * Math.cos(botHeading) - lateral * Math.sin(botHeading);
                double newLateral = axial * Math.sin(botHeading) + lateral * Math.cos(botHeading);

                double leftFrontPower  = newAxial + newLateral + yaw;
                double rightFrontPower = newAxial - newLateral - yaw;
                double leftBackPower   = newAxial - newLateral + yaw;
                double rightBackPower  = newAxial + newLateral - yaw;

                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

            }


            // --- G. 遥测数据显示 ---
            updateTelemetry(solution, targetAlliance);
        }

    }

    // =================================================================================================
    //                                     辅助方法
    // =================================================================================================

    /**
     * 获取当前机器人基于场地的航向（偏航角）
     * @return 校准后的场地航向角度（-180 到 180度）
     */
    private double getRobotFieldHeading() {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double correctedYaw = rawYaw + headingOffset;
        // 将角度标准化到 -180 到 180 度范围
        return normalizeAngle(correctedYaw);
    }

    /**
     * 将任意角度标准化到-180到180度之间
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * 计算航向误差（考虑角度环绕）
     */
    private double calculateHeadingError(double currentHeading, double targetHeading) {
        // 使用标准化函数确保两个角度都在同一范围内，再计算误差
        double error = normalizeAngle(targetHeading) - normalizeAngle(currentHeading);
        // 再次标准化误差，确保走最短路径
        return normalizeAngle(error);
    }

    /**
     * 执行自动转向到目标角度
     */
    private void performAutoTurn() {
        // --- 修改: 使用新的getRobotFieldHeading()方法 ---
        double currentHeading = getRobotFieldHeading();
        double headingError = currentHeading - targetHeading;

        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
        }

        double turnPower = calculatePIDOutput(headingError);
        setMecanumPower(turnPower);
    }


    // --- 其他辅助方法 (initializeHardware, resetPID, calculatePIDOutput, setMecanumPower, 等等) ---
    // 请将您原代码中的这些方法复制到此处，确保它们也使用 getRobotFieldHeading()
    // 为了简洁，这里只列出有修改或重要的方法。您需要将其他未变动的方法也复制过来。

    private void initializeHardware() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        block = hardwareMap.get(Servo.class, "Block");
        ls = hardwareMap.get(Servo.class,"ls");
        rs = hardwareMap.get(Servo.class,"rs");

        imu = hardwareMap.get(IMU.class,"imu");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "pinpoint");
        pinpointPoseProvider.initialize();

        archerLogic = new ArcherLogic();
    }


    private void updateTelemetry(LaunchSolution solution, String targetAlliance) {
        telemetry.addLine("--- Archer 系统状态 ---");
        telemetry.addData("底盘模式", isAutoAiming ? ">> 自动对准 <<" : "手动驾驶");
        telemetry.addData("目标联盟", "%s (按dpad_left/dpad_right切换)", targetAlliance);

        telemetry.addLine("\n--- 瞄准信息 ---");
        if (solution != null) {
            telemetry.addData("目标偏航角", "%.2f deg", solution.aimAzimuthDeg);
            telemetry.addData("当前场地偏航角", "%.2f deg", getRobotFieldHeading());
            telemetry.addData("发射电机转速 (RPM)", "%.0f", TARGET_RPM);
        } else {
            telemetry.addLine(">> 目标超出射程或无解 <<");
        }

        telemetry.addLine("\n--- 机器人状态 ---");
        telemetry.addData("归一化坐标", String.format(Locale.US, "{X: %.3f, Y: %.3f}", -pinpointPoseProvider.getX(DistanceUnit.CM)/365.76, pinpointPoseProvider.getY(DistanceUnit.CM)/365.76));
        telemetry.addData("IMU偏移量", "%.2f deg", headingOffset);
        telemetry.addData("ErrorHeading",previousError);
        telemetry.addData("Tatget",targetHeading);


        telemetry.update();
    }


    // 其他未修改的方法，如 resetPID, calculatePIDOutput, stopMotors, formatPose, rpmToTicksPerSecond 等，请从之前的代码中复制过来
    private void resetPID() {
        integralSum = 0;
        previousError = calculateHeadingError(getRobotFieldHeading(), targetHeading);
        previousTime = System.nanoTime();
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
        output = Range.clip(output, -TURN_POWER, TURN_POWER);
        if (Math.abs(error) > 2.0 && Math.abs(output) < 0.15) {
            output = 0.15 * Math.signum(output);
        }
        previousError = error;
        previousTime = currentTime;
        return output;
    }

    private void setMecanumPower(double turn) {
        double leftFrontPower = + turn;
        double rightFrontPower = - turn;
        double leftBackPower = + turn;
        double rightBackPower = - turn;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    private String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO) / 60.0;
    }
}