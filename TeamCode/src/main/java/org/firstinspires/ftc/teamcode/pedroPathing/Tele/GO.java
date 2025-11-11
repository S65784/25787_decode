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

import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;

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


    private boolean lastYState = false;
    private boolean state = false;
    public static final double MOTOR_TICK_COUNT = 28;

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


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Algorithm.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Algorithm.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Algorithm.P, Algorithm.I, Algorithm.D, Algorithm.F);
        Algorithm.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Algorithm.ls.setPosition(0.4);
        Algorithm.rs.setPosition(0.6);

        // 初始化PID时间戳
        previousTime = System.nanoTime();

        //----------------------------------------------
        runtime.reset();
        Algorithm.imu.resetYaw();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            double botHeading = Algorithm.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double newAxial = axial * Math.cos(botHeading) - lateral * Math.sin(botHeading);
            double newLateral = axial * Math.sin(botHeading) + lateral * Math.cos(botHeading);

            double leftFrontPower = newAxial + newLateral + yaw;
            double rightFrontPower = newAxial - newLateral - yaw;
            double leftBackPower = newAxial - newLateral + yaw;
            double rightBackPower = newAxial + newLateral - yaw;

            if (gamepad1.left_bumper) {
                Algorithm.imu.resetYaw();
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            Algorithm.leftFrontDrive.setPower(leftFrontPower);
            Algorithm.rightFrontDrive.setPower(rightFrontPower);
            Algorithm.leftBackDrive.setPower(leftBackPower);
            Algorithm.rightBackDrive.setPower(rightBackPower);
            // 将目标RPM转换为每秒的编码器刻度数
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            Algorithm.shooter.setVelocity(targetTicksPerSecond);

            if (gamepad1.a) {
                state = false;
                Algorithm.block.setPosition(0.3);
                Algorithm.intake.setPower(0.8);
                Algorithm.blender.setPower(0);
                TARGET_RPM = 1500;
            }

            if (gamepad1.b) {
                Algorithm.intake.setPower(-0.8);
                Algorithm.blender.setPower(-0.55);
                Algorithm.shooter.setPower(0);
            }
            if (gamepad1.x) {
                Algorithm.intake.setPower(0);
                Algorithm.blender.setPower(0);
                TARGET_RPM = 1500;
            }

//            if(gamepad1.dpad_right){
//                TARGET_RPM = 2950;
//                ERROR_RANGE = 100;
//                //超远
//            }
//            if(gamepad1.dpad_left){
//                TARGET_RPM = 1900;
//                ERROR_RANGE = 100;
//                //三角腰
//            }
//            if(gamepad1.dpad_down){
//                TARGET_RPM = 1700;
//                ERROR_RANGE = 100;
//                //三角底部
//            }
//            if(gamepad1.dpad_up){
//                TARGET_RPM = 2300;
//                ERROR_RANGE = 50;
//                //三角顶点
//            }

            // 如果正在自动转向，执行转向控制

            // if (isTurningToTarget) {
            //     performAutoTurn();
            // }


            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState) {
                state = true;
            }
            Algorithm.Shoot(3000, 30, state);

            lastYState = currentYState;

            if (gamepad1.dpad_down) {
                Algorithm.block.setPosition(0.35);
            }
            if (gamepad1.dpad_up) {
                Algorithm.block.setPosition(0.9);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", TARGET_RPM);
            telemetry.update();

        }
    }
}