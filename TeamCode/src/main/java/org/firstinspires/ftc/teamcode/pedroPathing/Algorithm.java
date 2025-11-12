package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Algorithm {
    public static IMU imu;
    public static DcMotor leftFrontDrive = null;
    public static DcMotor leftBackDrive = null;
    public static DcMotor rightFrontDrive = null;
    public static DcMotor rightBackDrive = null;
    public static DcMotor intake = null;
    public static DcMotor blender = null;
    public static DcMotorEx shooter = null;

    public static Servo block = null;
    public static Servo ls = null;
    public static Servo rs = null;




    public static int MOTOR_TICK_COUNT = 28;
    public Algorithm(HardwareMap hardwareMap) {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        block = hardwareMap.get(Servo.class, "Block");
        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");

        imu = hardwareMap.get(IMU.class, "imu");
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
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public static boolean readyShoot(int RPM, int error){
//
//        double currentVelocityTicks = shooter.getVelocity();
//        int MOTOR_TICK_COUNT = 28;
//        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
//        boolean volocitycheck = false;
//        if((RPM+error)>currentRPM && currentRPM>(RPM-error)) {
//            volocitycheck = true;
//        }
//        else {
//            volocitycheck = false;
//        }
//        return volocitycheck;
//    }
    public static double getCurrentRPM(){
        double currentVelocityTicks = shooter.getVelocity();
        //int MOTOR_TICK_COUNT = 28;
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        return currentRPM;
    }
    public static void Shoot(int targetRPM, int error, boolean state){
        if(state) {
            shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //double currentVelocityTicks = shooter.getVelocity();
            //int MOTOR_TICK_COUNT = 28;
            double targetTicksPerSecond = targetRPM * MOTOR_TICK_COUNT / 60;
            double currentRPM = getCurrentRPM();
            boolean volocitycheck = false;
            if ((targetRPM + error) > currentRPM && currentRPM > (targetRPM - error)) {
                volocitycheck = true;
            } else {
                volocitycheck = false;
            }
            if (!volocitycheck) {
                block.setPosition(1);
                intake.setPower(0);
                blender.setPower(0);
            } else if (volocitycheck) {
                block.setPosition(1);
                intake.setPower(1);
                blender.setPower(0.55);
            }
            shooter.setVelocity(targetTicksPerSecond);
        }

    }
    public static void Draw(boolean intakeState) {
        if (intakeState = true) {
            Algorithm.state = false;
            Algorithm.block.setPosition(0.3);
            Algorithm.intake.setPower(0.8);
            Algorithm.blender.setPower(0);
            Shoot(1500, 50, false);
        }
        if (intakeState = false) {
            Algorithm.intake.setPower(0);
            Algorithm.blender.setPower(0);
            Shoot(1500, 50, false);
        }

    }

    public static void servoControl(boolean servoState) {
        if (servoState = true) {
            Algorithm.ls.setPosition(0.4);
            Algorithm.rs.setPosition(0.6);
        }
    }

    public static boolean flag(boolean gamepad){
        boolean currentState = gamepad;
        boolean lastState = false;
        if (currentState && !lastState) {
            state = !state;
        }
        //Algorithm.Shoot(TARGET_RPM, ERROR_RANGE, Algorithm.state);
        lastState = currentState;
        return state;
    }


    public static double P = 140, I = 20, D = 33, F = 14.5;//p=140
    public static boolean lastYState = false;
    public static boolean state = false;


}

