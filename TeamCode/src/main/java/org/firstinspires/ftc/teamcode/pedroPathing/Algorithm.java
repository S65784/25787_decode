package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

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
    public static final int TARGET_RPM_YI = 3600;
    public static final int ERROR_RANGE_YI =500;
    public static final double SERVO_POSITION_YI = 0.5;
    public static final double BLENDER_POWER_YI = 1;
    public static final double INTAKE_POWER_YI = 1;

    public static final int TARGET_RPM_ER = 3500;
    public static final int ERROR_RANGE_ER = 357;
    public static final double SERVO_POSITION_ER = 0.5;
    public static final double BLENDER_POWER_ER = 1;
    public static final double INTAKE_POWER_ER = 1;

    public static final int TARGET_RPM_SAN = 4000;//2300
    public static final int ERROR_RANGE_SAN = 500;
    public static final double SERVO_POSITION_SAN = 0.523;
    public static final double BLENDER_POWER_SAN = 1;
    public static final double INTAKE_POWER_SAN = 1;

    public static final int TARGET_RPM_SI = 5250;//2950
    public static final int ERROR_RANGE_SI = 340;
    public static final double SERVO_POSITION_SI = 0.592;
    public static final double BLENDER_POWER_SI = 0.524;
    public static final double INTAKE_POWER_SI = 0.56;


    public static int MOTOR_TICK_COUNT = 28;



    public Algorithm(HardwareMap hardwareMap) {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");



        //block = hardwareMap.get(Servo.class, "Block");
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
        blender.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static double getCurrentRPM() {
        double currentVelocityTicks = shooter.getVelocity();
        //int MOTOR_TICK_COUNT = 28;
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        return currentRPM;
    }

    public static int targetRPM = 0;
    public static double currentRPM;
    public static boolean test = false;
    public static boolean shootState = false;

    public static void setRPM(int target_RPM){
        double targetTicksPerSecond = target_RPM * MOTOR_TICK_COUNT / 60;
        shooter.setVelocity(targetTicksPerSecond);
        targetRPM = target_RPM;
    }

    public static void shooterPID(){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public static void shoot(int target_RPM, int error,double blenderPower, boolean state, boolean yState) {
        if (state) {
            shooterPID();
            setRPM(target_RPM);
            if (yState) {
                if ((target_RPM + error) > currentRPM && currentRPM > (target_RPM - error)) {
                    intake.setPower(1);
                    blender.setPower(blenderPower);
                } else {
                    stopShoot(100);
                }
            }
        }
    }

    static ElapsedTime shootTimer = new ElapsedTime();

    public static void shootTime(int target_RPM, int error,double blenderPower, boolean state, boolean yState, int millitime) {
        shootTimer.reset();
        while (shootTimer.milliseconds() < millitime) {
            shoot(target_RPM, error,blenderPower ,state, yState);
            getCurrentRPM();
        }
        stopShoot();
    }
    public static void shootTime(int target_RPM, int error,double blenderPower ,boolean state, int millitime) {
        shootTimer.reset();
        while (shootTimer.milliseconds() < millitime) {
            shoot(target_RPM, error, blenderPower,state, true);
        }
    }
    public static ElapsedTime drawTimer = new ElapsedTime();
    public static void stopShoot(int millitime) {
        drawTimer.reset();
        while (drawTimer.milliseconds() < millitime) {
            intake.setPower(0);
            blender.setPower(0);
        }
    }

    public static void stopShoot() {
        intake.setPower(0);
        blender.setPower(0);
        isChecked = false;
    }

    public static void draw() {
        state = false;
        //block.setPosition(0.3);
        intake.setPower(1);
        blender.setPower(0);
        //shoot(Algorithm.TARGET_RPM_YI,Algorithm.ERROR_RANGE_YI,true);
        test = true;
    }


    static ElapsedTime preShooterTimer = new ElapsedTime();
    public static void preShooterMove(int millitime, double power) {
            shootState = true;
            preShooterTimer.reset();
        while (preShooterTimer.milliseconds() < millitime && shootState)  blender.setPower(power);
        blender.setPower(0);

    }
    public static void keep(double power) {
        intake.setPower(power);
    }

    public static void keep() {
        intake.setPower(0.12);
    }
    public static void preShooterMove(int millitime) {
        shootState = true;
        preShooterTimer.reset();
        while(preShooterTimer.milliseconds() < millitime && shootState)  blender.setPower(0.53);
        blender.setPower(0);

    }
    public static void preShooterMove() {
        shootState = true;
        preShooterTimer.reset();
        if(preShooterTimer.milliseconds() < 420 && shootState)  blender.setPower(0.53);
        else  blender.setPower(0);

    }

    public static void servoControl() {
        ls.setPosition(0.5);
        rs.setPosition(0.5);
    }

    private static boolean lastState = false;
    private static boolean flagState = false;

    public static boolean flag(boolean gamepad) {
        //boolean currentState = gamepad;
        if (gamepad && !lastState) {
            flagState = !flagState;
        }
        lastState = gamepad;
        return flagState;
    }
    public static ShootMode shootMode1 = new ShootMode(TARGET_RPM_YI, ERROR_RANGE_YI, SERVO_POSITION_YI,BLENDER_POWER_YI,INTAKE_POWER_YI);
    public static ShootMode shootMode2 = new ShootMode(TARGET_RPM_ER, ERROR_RANGE_ER, SERVO_POSITION_ER,BLENDER_POWER_ER,INTAKE_POWER_ER);
    public static ShootMode shootMode3 = new ShootMode(TARGET_RPM_SAN, ERROR_RANGE_SAN, SERVO_POSITION_SAN,BLENDER_POWER_SAN,INTAKE_POWER_SAN);
    public static ShootMode shootMode4 = new ShootMode(TARGET_RPM_SI, ERROR_RANGE_SI, SERVO_POSITION_SI,BLENDER_POWER_SI,INTAKE_POWER_SI);
//
    public static void shootOpenLoop(int target_RPM,double blenderPower,double intakePower ,boolean state, boolean yState) {
        if (state) {
            shooterPID();
            if (yState) {
                intake.setPower(intakePower);
                blender.setPower(blenderPower);
            }
            setRPM(target_RPM);
        }


    }
    public static boolean checkRPM(int targetRPM,int error){
        double current_RPM = getCurrentRPM();
        currentRPM = current_RPM;
        if ((targetRPM + error) > currentRPM && currentRPM > (targetRPM - error)){
            return true;
        }else{
            return false;
        }
    }
    static boolean isChecked = false;
    public static void shootCheckOnce(int target_RPM,int error,double blenderPower,double intakePower,boolean state, boolean yState){
        if(state) {
            if (yState) {
                if (checkRPM(target_RPM, error)) {
                    isChecked = true;
                }
                if (isChecked) {
                    intake.setPower(intakePower);
                    blender.setPower(blenderPower);
                }
            }
            setRPM(target_RPM);
        }
    }

    public static void shootCheckOnceTime(int target_RPM,int error,double blenderPower,double intakePower,boolean state, boolean yState,int milli){
        shootTimer.reset();
        while (shootTimer.milliseconds() < milli) {
            shootCheckOnce(target_RPM, error,blenderPower ,intakePower,state, yState);
        }
        stopShoot();
        isChecked = false;
    }

    public static void sleep(long ms){
            try {
                Thread.sleep(ms);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

    }

    public static void sleepForAWhile(long ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }
    public static void sleepForAWhile(){
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

        public static double P = 125, I = 0.4, D = 0.1, F = 16.27;
//    public static double P = 140, I = 20, D = 33, F = 14.5;
        public static boolean lastYState = false;
        public static boolean state = false;

}