package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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




    //constants
    public static final int TARGET_RPM_YI = 1900;
    public static final int ERROR_RANGE_YI = 50;

    public static final int TARGET_RPM_ER = 2200;
    public static final int ERROR_RANGE_ER = 100;

    public static final int TARGET_RPM_SAN = 2700;//2300
    public static final int ERROR_RANGE_SAN = 50;

    public static final int TARGET_RPM_SI = 3500;//2950
    public static final int ERROR_RANGE_SI = 34;



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

    public static double getCurrentRPM(){
        double currentVelocityTicks = shooter.getVelocity();
        //int MOTOR_TICK_COUNT = 28;
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        return currentRPM;
    }
    public static int targetRPM = 0;
    public static double currentRPM;
    public static boolean test = false;


    public static void shoot(int target_RPM, int error, boolean state, boolean yState){
        if(state) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //double currentVelocityTicks = shooter.getVelocity();
            //int MOTOR_TICK_COUNT = 28;
            double targetTicksPerSecond = target_RPM * MOTOR_TICK_COUNT / 60;
            double current_RPM = getCurrentRPM();
            currentRPM = current_RPM;

            if(yState) {
                if ((target_RPM + error) > currentRPM && currentRPM > (target_RPM - error)) {
                    //block.setPosition(1);
                    intake.setPower(1);
                    blender.setPower(0.55);
                } else {
                    stopShoot();
                }
            }

            shooter.setVelocity(targetTicksPerSecond);
            targetRPM = target_RPM;
        }
    }

    static ElapsedTime shootTimer = new ElapsedTime();

    public static void shootTime(int target_RPM, int error, boolean state, int time) {
        shootTimer.reset();
        while (shootTimer.milliseconds() < time) {
            shoot(target_RPM,error,state,true);
        }
    }
    public static void stopShoot(){
        intakeState = false;
        block.setPosition(1);
        intake.setPower(0);
        blender.setPower(0);
        test = false;
    }

    static ElapsedTime intakeStopTimer = new ElapsedTime();

    public static void stopShoot(int time) {
        intakeStopTimer.reset();
        while (intakeStopTimer.milliseconds() < time) {
            draw();
        }
    }

    public static ElapsedTime drawTimer = new ElapsedTime();
    public static void drawTime(int time) {
        drawTimer.reset();
        while (drawTimer.milliseconds() < time) {
            draw();
        }
    }
    public static void draw() {
            state = false;
            block.setPosition(0.3);
            intake.setPower(0.8);
            blender.setPower(0);
            test = true;
    }

    public static void servoControl() {
            ls.setPosition(0.5);
            rs.setPosition(0.5);
//        ls.setPosition(0.4);
//        rs.setPosition(0.6);
    }
    private static boolean lastState = false;
    private static boolean flagState = false;
    public static boolean flag(boolean gamepad){
        //boolean currentState = gamepad;
        if (gamepad && !lastState) {
            flagState = !flagState;
        }
        lastState = gamepad;
        return flagState;
    }

    public static void shootMode(int mode,boolean yState){
        ShootMode shootMode1 = new ShootMode(TARGET_RPM_YI,ERROR_RANGE_YI);
        ShootMode shootMode2 = new ShootMode(TARGET_RPM_ER,ERROR_RANGE_ER);
        ShootMode shootMode3 = new ShootMode(TARGET_RPM_SAN,ERROR_RANGE_SAN);
        ShootMode shootMode4 = new ShootMode(TARGET_RPM_SI,ERROR_RANGE_SI);
        if(mode==1){
            shootMode1.shoot(yState);
        }else if(mode==2){
            shootMode2.shoot(yState);
        }else if(mode==3){
            shootMode3.shoot(yState);
        }else if(mode==4){
            shootMode4.shoot(yState);
        }
    }

    public static void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
//    public static Runnable shoot(Runnable shootingMethod){
//        //general
//        shootingMethod.run();
//        return null;
//    }
//    public static void test(){
//        shoot(()->{
//
//        });
//    }
    public static double P = 135, I = 0, D = 80, F = 14;
//    public static double P = 140, I = 20, D = 33, F = 14.5;
    public static boolean lastYState = false;
    public static boolean state = false;
    public static boolean intakeState = false;


}

