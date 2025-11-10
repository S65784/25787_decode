package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class shooterTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
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
    public static double P = 140, I = 0, D = 33, F = 14.5;
    public static double TARGET_RPM = 0;
    //-----------------------------------------------------------------------
    // 控制参数
    private double targetHeading = 0;
    private boolean isTurningToTarget = false;
    private final double TURN_POWER = 0.5;
    private final double HEADING_THRESHOLD = 2.0; // 角度阈值（度）
    private final double P_TURN_GAIN = 0.008; // 转向比例增益

    // PID控制参数
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.001; // 积分增益
    private final double D_GAIN = 0.005; // 微分增益

    // 目标角度常量
    private final double TARGET_ANGLE_LEFT = -45.0;  // 左转45度
    private final double TARGET_ANGLE_RIGHT = 45.0;  // 右转45度
    @Override
    public void runOpMode() {

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
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        ls.setPosition(0.4);
        rs.setPosition(0.6);
        //---------------------------------------------
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.resetYaw();
        sleep(500);
        telemetry.addData("状态", "IMU偏航角已重置");
        telemetry.addData("重置后航向", "%.1f°", getHeading());
        telemetry.update();

        // 初始化PID时间戳
        previousTime = System.nanoTime();

        //----------------------------------------------
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // 将目标RPM转换为每秒的编码器刻度数
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            shooter.setVelocity(targetTicksPerSecond);

            if(gamepad1.a){
                block.setPosition(0.3);
                intake.setPower(0.8);
                blender.setPower(0);
                TARGET_RPM = 1500;
            }

            if(gamepad1.b){
                intake.setPower(-0.8);
                //blender.setPower(-0.55);
                shooter.setPower(0);
            }
            if(gamepad1.x){
                shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake.setPower(0);
                blender.setPower(0);
                TARGET_RPM = 0;
            }

            if(gamepad1.dpad_right){//低
                block.setPosition(0.3);
            }
            if(gamepad1.dpad_left){
                block.setPosition(1);
            }
            checkAutoTurnTriggers();

            // 如果正在自动转向，执行转向控制
            if (isTurningToTarget) {
                performAutoTurn();
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
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

            double currentVelocityTicks = shooter.getVelocity();
            double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
            boolean volocitycheck = false;
            if((TARGET_RPM+100)>currentRPM && currentRPM>(TARGET_RPM-100)) {
                volocitycheck = true;
            }
            else {
                volocitycheck = false;
            }

            boolean currentYState = gamepad1.y;

            if (currentYState && !volocitycheck) {
                block.setPosition(1);
                intake.setPower(0);
                blender.setPower(0);
                TARGET_RPM=2500;
                state = true;

            } else if (currentYState && volocitycheck) {
                block.setPosition(1);
                intake.setPower(1);
                blender.setPower(0.55);
                state = false;
            }

            lastYState = currentYState;

            if(gamepad1.dpad_down){
                block.setPosition(0.35);

            }
            if(gamepad1.dpad_up){
                block.setPosition(0.9);

            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", TARGET_RPM);
            telemetry.addData("当前 RPM", "%.2f", currentRPM);
            telemetry.update();


        }



    }

    private void checkAutoTurnTriggers() {
        // 左肩键 - 转向-45度
        if (gamepad1.left_bumper && !isTurningToTarget) {
            targetHeading = TARGET_ANGLE_LEFT;
            isTurningToTarget = true;
            // 重置PID状态
            resetPID();
            telemetry.addData("自动转向", "转向 -45 度");
        }

        // 右肩键 - 转向45度
        if (gamepad1.right_bumper && !isTurningToTarget) {
            targetHeading = TARGET_ANGLE_RIGHT;
            isTurningToTarget = true;
            // 重置PID状态
            resetPID();
            telemetry.addData("自动转向", "转向 45 度");
        }

        // 如果按下B键，取消自动转向
         /*if (gamepad1.b && isTurningToTarget) {
             isTurningToTarget = false;
             stopMotors();
             telemetry.addData("自动转向", "已取消");
         }

         // 如果按下Y键，重置IMU偏航角
         if (gamepad1.y) {
             resetIMU();
             telemetry.addData("IMU", "偏航角已重置");
         }*/
    }

    /**
     * 执行自动转向到目标角度
     */
    private void performAutoTurn() {
        double currentHeading = getHeading();
        double headingError = calculateHeadingError(currentHeading, targetHeading);

        // 如果误差在阈值内，停止转向
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            stopMotors();
            isTurningToTarget = false;
            telemetry.addData("自动转向", "完成");
            return;
        }

        // 计算转向功率（PID控制）
        double turnPower = calculatePIDOutput(headingError);

        // 执行转向（原地旋转）
        setMecanumPower(0, 0, turnPower);
    }

    /**
     * 计算PID控制输出
     * @param error 当前误差
     * @return PID控制输出（转向功率）
     */
    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9; // 转换为秒

        // 防止除零错误
        if (deltaTime == 0) {
            deltaTime = 0.01; // 默认10ms
        }

        // 比例项
        double proportional = P_TURN_GAIN * error;

        // 积分项（带积分限幅防止积分饱和）
        integralSum += error * deltaTime;
        // 积分限幅
        double maxIntegral = 0.5 / I_GAIN; // 限制积分项的最大影响
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;

        // 微分项
        double derivative = D_GAIN * (error - previousError) / deltaTime;

        // 计算总输出
        double output = proportional + integral + derivative;

        // 限制输出范围
        output = Range.clip(output, -TURN_POWER, TURN_POWER);

        // 更新状态变量
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    /**
     * 重置PID控制器状态
     */
    private void resetPID() {
        previousError = 0;
        integralSum = 0;
        previousTime = System.nanoTime();
    }

    /**
     * 手动控制麦克纳姆轮移动
     */
    private void manualMecanumDrive() {
        // 从游戏板读取输入
        double drive = -gamepad1.left_stick_y;  // 前后移动（摇杆上下）
        double strafe = gamepad1.left_stick_x;  // 左右平移（摇杆左右）
        double turn = gamepad1.right_stick_x;   // 旋转（右摇杆左右）

        // 应用死区处理小摇杆输入
        if (Math.abs(drive) < 0.1) drive = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(turn) < 0.1) turn = 0;

        // 设置电机功率
        setMecanumPower(drive, strafe, turn);
    }

    /**
     * 设置麦克纳姆轮功率
     * @param drive 前后移动功率 (-1 到 1)
     * @param strafe 左右平移功率 (-1 到 1)
     * @param turn 旋转功率 (-1 到 1)
     */
    private void setMecanumPower(double drive, double strafe, double turn) {
        // 麦克纳姆轮功率计算
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // 归一化功率值，确保不超过±1.0
        double maxPower = Math.max(Math.max(
                        Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // 设置电机功率
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }

    /**
     * 停止所有电机
     */
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * 计算航向误差（考虑角度环绕）
     * @param currentHeading 当前航向
     * @param targetHeading 目标航向
     * @return 标准化后的误差（-180 到 180度）
     */
    private double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;

        // 标准化误差到 -180 到 180 度范围
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    /**
     * 获取当前机器人航向（偏航角）
     * @return 当前航向角度（度）
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void resetIMU() {
        imu.resetYaw();
        // 给IMU一点时间处理重置
        sleep(50);
    }
    /**
     * 更新遥测数据显示
     */
    private void updateTelemetry() {
        double currentHeading = getHeading();
        double error = calculateHeadingError(currentHeading, targetHeading);

        telemetry.addData("当前航向", "%.1f°", currentHeading);
        telemetry.addData("目标航向", "%.1f°", targetHeading);
        telemetry.addData("航向误差", "%.1f°", error);
        telemetry.addData("自动转向状态", isTurningToTarget ? "进行中" : "关闭");
        telemetry.addData("PID状态", "P:%.4f I:%.4f D:%.4f",
                P_TURN_GAIN * error, I_GAIN * integralSum,
                D_GAIN * (error - previousError));
        telemetry.addData("控制说明",
                "左肩键:-45° | 右肩键:+45° | B键:取消转向 | Y键:重置IMU");
        telemetry.update();


    }
}
