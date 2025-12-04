package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.PinpointPoseProvider;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.ArcherLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.CalculationParams;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.LaunchSolution;

import java.util.Locale;

public class VisionAlgorithm {
    public static AprilTagLocalizer aprilTagLocalizer;
    public static IPoseProvider pinpointPoseProvider;
    public static ArcherLogic archerLogic;
    public static final double SHOOTER_TICKS_PER_REV = 28;
    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static double targetHeading = 0;
    public static final double TURN_POWER = 1.0;
    public static final double HEADING_THRESHOLD = 1.5;
    public static final double P_TURN_GAIN = 0.1;
    public static final double I_GAIN = 0.0001;
    public static final double D_GAIN = 0.003;
    public static double integralSum = 0;
    public static double previousError = 0;
    public static long previousTime = 0;
    public boolean isAutoAiming = false;
    public boolean rightBumperLastState = false;
    public static double headingOffset = 0.0;


    static double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
    static double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);
    static double normalizationX = robotX_cm / 365.76;
    static double normalizationY = robotY_cm / 365.76;
    static double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
    static double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
    static double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
    static double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
    public static double direction_deg = Math.toDegrees(direction_rad);
    boolean s = direction_deg<0;



    public static double getRobotFieldHeading() {
        double rawYaw = Algorithm.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double correctedYaw = rawYaw + headingOffset;
        // 将角度标准化到 -180 到 180 度范围
        return normalizeAngle(correctedYaw);
    }

    /**
     * 将任意角度标准化到-180到180度之间
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * 计算航向误差（考虑角度环绕）
     */
    public static double calculateHeadingError(double currentHeading, double targetHeading) {
        // 使用标准化函数确保两个角度都在同一范围内，再计算误差
        double error = normalizeAngle(targetHeading) - normalizeAngle(currentHeading);
        // 再次标准化误差，确保走最短路径
        return normalizeAngle(error);
    }

    /**
     * 执行自动转向到目标角度
     */
    public static void performAutoTurn() {
        // --- 修改: 使用新的getRobotFieldHeading()方法 ---
        double currentHeading = getRobotFieldHeading();
        double headingError = currentHeading - targetHeading;

        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
        }

        double turnPower = calculatePIDOutput(headingError);
        setMecanumPower(0, 0, turnPower);
    }


    // --- 其他辅助方法 (initializeHardware, resetPID, calculatePIDOutput, setMecanumPower, 等等) ---
    // 请将您原代码中的这些方法复制到此处，确保它们也使用 getRobotFieldHeading()
    // 为了简洁，这里只列出有修改或重要的方法。您需要将其他未变动的方法也复制过来。


//    private void updateTelemetry(LaunchSolution solution, String targetAlliance) {
//        telemetry.addLine("--- Archer 系统状态 ---");
//        telemetry.addData("底盘模式", isAutoAiming ? ">> 自动对准 <<" : "手动驾驶");
//        telemetry.addData("目标联盟", "%s (按X/B切换)", targetAlliance);
//
//        telemetry.addLine("\n--- 瞄准信息 ---");
//        if (solution != null) {
//            telemetry.addData("目标偏航角", "%.2f deg", solution.aimAzimuthDeg);
//            telemetry.addData("当前场地偏航角", "%.2f deg", getRobotFieldHeading());
//            telemetry.addData("发射电机转速 (RPM)", "%.0f", TARGET_RPM);
//        } else {
//            telemetry.addLine(">> 目标超出射程或无解 <<");
//        }
//
//        telemetry.addLine("\n--- 机器人状态 ---");
//        telemetry.addData("归一化坐标", String.format(Locale.US, "{X: %.3f, Y: %.3f}", -pinpointPoseProvider.getX(DistanceUnit.CM)/365.76, pinpointPoseProvider.getY(DistanceUnit.CM)/365.76));
//        telemetry.addData("IMU偏移量", "%.2f deg", headingOffset);
//        telemetry.addData("ErrorHeading",previousError);
//        telemetry.addData("Tatget",targetHeading);
//
//
//        telemetry.update();
//    }


    // 其他未修改的方法，如 resetPID, calculatePIDOutput, stopMotors, formatPose, rpmToTicksPerSecond 等，请从之前的代码中复制过来
    public static void resetPID() {
        integralSum = 0;
        previousError = calculateHeadingError(getRobotFieldHeading(), targetHeading);
        previousTime = System.nanoTime();
    }

    public static double calculatePIDOutput(double error) {
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

    public static void setMecanumPower(double drive, double strafe, double turn) {
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
        Algorithm.leftFrontDrive.setPower(leftFrontPower);
        Algorithm.rightFrontDrive.setPower(rightFrontPower);
        Algorithm.leftBackDrive.setPower(leftBackPower);
        Algorithm.rightBackDrive.setPower(rightBackPower);

    }

    public static String formatPose(Pose2D pose) {
        if (pose == null) return "null";
        return String.format(Locale.US, "X: %.2f cm, Y: %.2f cm, H: %.2f deg",
                pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM), pose.getHeading(AngleUnit.DEGREES));
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * SHOOTER_TICKS_PER_REV * SHOOTER_GEAR_RATIO) / 60.0;
    }

    static CalculationParams currentParams = new CalculationParams(normalizationX, normalizationY, speed_m_s, direction_deg, getAlliance());

    static LaunchSolution solution = archerLogic.calculateSolution(currentParams);
    public static void autoTurnControl(boolean state){
        if (state) {
            if (solution != null) {
                targetHeading = solution.aimAzimuthDeg;
                VisionAlgorithm.performAutoTurn();

            } else {

            }
        } else {
            //Algorithm.chassisDrive();
        }
    }

    static String alliance;
    public static void setAlliance(boolean state){
        if(state){
            alliance = "Blue";
        }else {
            alliance = "Red";
        }
    }


    public static String getAlliance(){
        return alliance;
    }

    public static void someMethodForVision(){
        Pose2D aprilTagInitialPose = VisionAlgorithm.aprilTagLocalizer.getRobotPose();
        if (aprilTagInitialPose != null) {
            VisionAlgorithm.pinpointPoseProvider.setPose(aprilTagInitialPose);

            // --- 核心修改: 计算并设置IMU偏移量 ---
            double aprilTagHeading = aprilTagInitialPose.getHeading(AngleUnit.DEGREES);
            double rawImuHeading = Algorithm.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            VisionAlgorithm.headingOffset = aprilTagHeading - rawImuHeading;


        } else {
            // 如果未找到AprilTag, 则将当前朝向作为0度 (与之前的resetYaw效果相同)

            Algorithm.imu.resetYaw(); // 仅在失败时重置，以保证有一个已知的起点
            VisionAlgorithm.headingOffset = 0; // 偏移量为0
        }

    }
}
