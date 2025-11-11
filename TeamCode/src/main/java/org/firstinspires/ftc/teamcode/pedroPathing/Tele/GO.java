package org.firstinspires.ftc.teamcode.pedroPathing.Tele;
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

public class GO extends LinearOpMode {
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
    public static double P = 140, I = 20, D = 33, F = 14.5;//p=140
    public static double TARGET_RPM = 0;
    public static double ERROR_RANGE;

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

        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

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


        telemetry.addData("状态", "IMU偏航角已重置");
        telemetry.addData("重置后航向", "%.1f°", getHeading());
        telemetry.update();

        // 初始化PID时间戳
        previousTime = System.nanoTime();

        //----------------------------------------------
        runtime.reset();
        imu.resetYaw();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // 将目标RPM转换为每秒的编码器刻度数
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            shooter.setVelocity(targetTicksPerSecond);

            if(gamepad1.a){
                state=false;
                block.setPosition(0.3);
                intake.setPower(0.8);
                blender.setPower(0);
                TARGET_RPM = 1500;
            }

            if(gamepad1.b){
                intake.setPower(-0.8);
                blender.setPower(-0.55);
                shooter.setPower(0);
            }
            if(gamepad1.x){
                intake.setPower(0);
                blender.setPower(0);
                TARGET_RPM = 1500;
            }

            if(gamepad1.dpad_right){
                TARGET_RPM = 2950;
                ERROR_RANGE = 100;
                //超远
            }
            if(gamepad1.dpad_left){
                TARGET_RPM = 1900;
                ERROR_RANGE = 100;
                //三角腰
            }
            if(gamepad1.dpad_down){
                TARGET_RPM = 1700;
                ERROR_RANGE = 100;
                //三角底部
            }
            if(gamepad1.dpad_up){
                TARGET_RPM = 2300;
                ERROR_RANGE = 50;
                //三角顶点
            }

            // 如果正在自动转向，执行转向控制

            // if (isTurningToTarget) {
            //     performAutoTurn();
            // }

            double max;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double newAxial = axial * Math.cos(botHeading) - lateral * Math.sin(botHeading);
            double newLateral = axial * Math.sin(botHeading) + lateral * Math.cos(botHeading);

            double leftFrontPower  = newAxial + newLateral + yaw;
            double rightFrontPower = newAxial - newLateral - yaw;
            double leftBackPower   = newAxial - newLateral + yaw;
            double rightBackPower  = newAxial + newLateral - yaw;

            if(gamepad1.left_bumper) {
                imu.resetYaw();
            }


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
            if((TARGET_RPM+ERROR_RANGE)>currentRPM && currentRPM>(TARGET_RPM-ERROR_RANGE)) {
                volocitycheck = true;
            }
            else {
                volocitycheck = false;
            }

            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState) {
                state = true;
            }

            if (state && !volocitycheck) {
                block.setPosition(1);
                intake.setPower(0);
                blender.setPower(0);
            }
            else if (state && volocitycheck) {
                block.setPosition(1);
                intake.setPower(1);
                blender.setPower(0.55);
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

            if(gamepad1.right_bumper) {
                isTurningToTarget = true;
                while (!isTurningToTarget){
                    performAutoTurn();}
            }
        }



    }


    // 左肩键 - 转向-45度



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


    private void performAutoTurn() {
        double currentHeading = getHeading();
        double headingError =currentHeading - targetHeading;

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

        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / I_GAIN;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;

        // 微分项
        double derivative = D_GAIN * (error - previousError) / deltaTime;

        double output = proportional + integral + derivative;

        //
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




    private void setMecanumPower(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        double maxPower = Math.max(Math.max(
                        Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }


    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void resetIMU() {
        imu.resetYaw();
        sleep(50);
    }

}