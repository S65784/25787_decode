package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
    public static DcMotorEx shooter2 = null;
    //public static DcMotorEx terrace = null;

    public static Servo ls = null;
    public static Servo rs = null;


    //constants
    public static final int TARGET_RPM_YI = 2700;//2450
    public static final int ERROR_RANGE_YI =500;
    public static final double SERVO_POSITION_YI = 0.56;
    public static final double BLENDER_POWER_YI = 1;
    public static final double INTAKE_POWER_YI = 1;

    public static final int TARGET_RPM_ER = 3000;//3000 3300
    public static final int ERROR_RANGE_ER = 200;
    public static final double SERVO_POSITION_ER = 0.73;
    public static final double BLENDER_POWER_ER = 1;
    public static final double INTAKE_POWER_ER = 1;

    public static final int TARGET_RPM_SAN = 3600;//2300
    public static final int ERROR_RANGE_SAN = 500;
    public static final double SERVO_POSITION_SAN = 0.77;
    public static final double BLENDER_POWER_SAN = 1;
    public static final double INTAKE_POWER_SAN = 1;

    public static final int TARGET_RPM_SI = 4500;//2950
    public static final int ERROR_RANGE_SI = 200;//205
    public static final double SERVO_POSITION_SI = 0.79;
    public static final double BLENDER_POWER_SI = 0.8;
    public static final double INTAKE_POWER_SI = 1;

    public static final double TERRACE_GEAR_RATIO = 1;

    public enum Alliance{
    RED(new Point(RED_GOAL_POSITION_X,RED_GOAL_POSITION_Y),RED_FAR_HEADING,20),
    BLUE(new Point(BLUR_GOAL_POSITION_X,BLUR_GOAL_POSITION_Y),BLUE_FAR_HEADING,24);

    private Point point;
    private double farTurretHeading;
    private int ID;

        Alliance(Point point, double farTurretHeading,int ID){
        this.point = point;
        this.farTurretHeading = farTurretHeading;
        this.ID = ID;
        }

        public Point getPoint(){
            return point;
        }
        public double getFarTurretHeading(){return farTurretHeading;}
        public int getID(){return ID;}
    }
    public static final int BLUR_GOAL_POSITION_X = 1400;//-1750
    public static final int BLUR_GOAL_POSITION_Y = -1400;//1750
    public static final int RED_GOAL_POSITION_X =1400;
    public static final int RED_GOAL_POSITION_Y =1400;
    public static final double RED_FAR_HEADING = 75;
    public static final double BLUE_FAR_HEADING = 180-75;




    public static int MOTOR_TICK_COUNT = 28;

    public static double P = 0.005, I = 0, D = 0;
    public static double F = 0.0004;
    public static PID pid = new PID(P,0,D);

//    public static double P = 90, I = 0, D = 1, F = 13;
    public static double dP = 125, dI = 0.4, dD = 0.1, dF = 16.27;//da
    //        public static double P = 125, I = 0.4, D = 0.1, F = 16.27;
//    public static double P = 140, I = 20, D = 33, F = 14.5;
    public static boolean lastYState = false;
    public static boolean state = false;

    public static PIDFCoefficients pidClose = new PIDFCoefficients(P, I, D, F);
    public static PIDFCoefficients pidFar = new PIDFCoefficients(dP, dI, dD, dF);



    public Algorithm(HardwareMap hardwareMap) {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooter2 = hardwareMap.get(DcMotorEx.class, "ShooterR");


        //block = hardwareMap.get(Servo.class, "Block");
        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static double getCurrentRPM() {
        double currentVelocityTicks = shooter.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        return currentRPM;
    }

    public static int targetRPM = 0;
    public static double targetPower = 0;
    public static double currentRPM;
    public static boolean test = false;
    public static boolean shootState = false;
    public static double pidPower;
    public static double targetTicksPerSecond;

    public static void setRPM(int target_RPM){
        targetTicksPerSecond = target_RPM * MOTOR_TICK_COUNT / 60;
        targetRPM = target_RPM;

    }

    public static void updateRPM(){
        double feedForward = F * targetTicksPerSecond;
        pidPower = pid.update(targetTicksPerSecond-shooter.getVelocity());
        double target_Power = Range.clip(feedForward + pidPower, -1.0, 1.0);
        shooter.setPower(target_Power);
        shooter2.setPower(target_Power);
        targetPower = target_Power;
    }

//    public static void shooterPID(){
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
//        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//    }
//    public static void shooterPIDFar(){
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(dP, dI, dD, dF);
//        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//    }

    public static void shoot(int target_RPM, int error,double blenderPower, boolean state, boolean yState) {
        if (state) {
            setRPM(target_RPM);
            currentRPM = getCurrentRPM();
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
    public static void shoot(int target_RPM, int error,double blenderPower, boolean state, boolean yState, boolean iyu) {
        if (state) {
            //shooterPIDFar();
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
    static boolean isStart = false;


    public static void shootTime(int target_RPM, int error,double blenderPower, boolean state, boolean yState, int millitime) {
        if(!isStart){
            shootTimer.reset();
            isStart = true;
        }
        if (shootTimer.milliseconds() < millitime) {
            shoot(target_RPM, error,blenderPower ,state, yState);
            getCurrentRPM();
        }else {
            stopShoot();
            isStart = false;
        }
    }

    public static boolean shootTimeCheck(){
        return !isStart;
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
        blender.setPower(-0.81);
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
    public static void preShooterMove(int millitime) {
        shootState = true;
        preShooterTimer.reset();
        while(preShooterTimer.milliseconds() < millitime && shootState)  blender.setPower(-0.81);
        blender.setPower(0);

    }
    public static void preShooterMove() {
        shootState = true;
        preShooterTimer.reset();
        if(preShooterTimer.milliseconds() < 312 && shootState)  blender.setPower(-0.81);//420, 0.53
        else  blender.setPower(0);

    }

    public static void preShooterMove_Blue3() {
        shootState = true;
        preShooterTimer.reset();
        if(preShooterTimer.milliseconds() < 100 && shootState)  blender.setPower(-0.1);//420, 0.53
        else  blender.setPower(0);

    }

    public static void keep(double power) {intake.setPower(power);}
    public static void keep() {
        intake.setPower(0.12);
    }
    public static void reverseBlender(double power) {blender.setPower(power);}
    public static void reverseBlender() {
        blender.setPower(-0.81);
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
    public static ShootMode shootMode1 = new ShootMode(TARGET_RPM_YI, ERROR_RANGE_YI, SERVO_POSITION_YI,BLENDER_POWER_YI,INTAKE_POWER_YI,pidClose);
    public static ShootMode shootMode2 = new ShootMode(TARGET_RPM_ER, ERROR_RANGE_ER, SERVO_POSITION_ER,BLENDER_POWER_ER,INTAKE_POWER_ER,pidClose);
    public static ShootMode shootMode3 = new ShootMode(TARGET_RPM_SAN, ERROR_RANGE_SAN, SERVO_POSITION_SAN,BLENDER_POWER_SAN,INTAKE_POWER_SAN,pidClose);
    public static ShootMode shootMode4 = new ShootMode(TARGET_RPM_SI, ERROR_RANGE_SI, SERVO_POSITION_SI,BLENDER_POWER_SI,INTAKE_POWER_SI,pidFar);
//
    public static void shootOpenLoop(int target_RPM,double blenderPower,double intakePower ,boolean state, boolean yState) {
        if (state) {
            //shooterPID();
            if (yState) {
                intake.setPower(intakePower);
                blender.setPower(blenderPower);
            }
            setRPM(target_RPM);
        }

    }

    public static void update(){
        pidPower = pid.update(targetTicksPerSecond-shooter.getVelocity());
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
            updateRPM();
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
}