package org.firstinspires.ftc.teamcode.pedroPathing.Tele;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.TurretAlgorithm;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;


@TeleOp(name = "AAAHardyGameRED-Tres")
public class CUATRO extends LinearOpMode {
    private Algorithm algorithm;
    private TurretAlgorithm turretAlgorithm;
    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lighttime = new ElapsedTime();

    private IMU imu;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotor blender = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;

    private Servo ls = null;
    private Servo rs = null;
    private Servo tr;
    private Servo tl;
    private double currentPos = 0.1;      // 初始位置（中位）
    private double sensitivity = 0.00008;  // 灵敏度：tx 每偏一度，舵机移动的比例
    private double deadzone = 0.5;


    private DistanceSensor distanceSensor;
    private Servo headlight;



    private final double TURRET_CENTER_POSITION = 0.1;

    private boolean lastYState = false;
    private boolean state = false;
    public static final double MOTOR_TICK_COUNT = 28;

    double forwardmillitime;
    double reversemillitime;

    public static double reversePower = 0;
    public static boolean reverseState = false;
    public static boolean timeState = false;

    public static double P = 0.005, I = 0, D = 0;
    public static double F = 0.0004;
    public static double TARGET_RPM = 0;
    public static double ErrorRangeStart = 200;
    public static double ErrorRangeEnd = 200;
    public static double servoPosition = 0.47;
    public static boolean normalState = true;
    public static double blenderPower = 0;
    public static boolean check = false;



    @Override
    public void runOpMode() {
        ElapsedTime drawTime = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        algorithm = new Algorithm(hardwareMap);
        turretAlgorithm = new TurretAlgorithm(hardwareMap,telemetry, Algorithm.Alliance.RED,follower);

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooter2 = hardwareMap.get(DcMotorEx.class, "ShooterR");

        // block = hardwareMap.get(Servo.class, "Block");
        ls = hardwareMap.get(Servo.class,"ls");
        rs = hardwareMap.get(Servo.class,"rs");
        //tr = hardwareMap.get(Servo.class,"tr");
        //tl = hardwareMap.get(Servo.class,"tl");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "Ball");
        Servo headlight = hardwareMap.get(Servo.class, "Indicator1");//

        //tl = hardwareMap.get(Servo.class, "tl");
        //tr = hardwareMap.get(Servo.class, "tr");


        imu = hardwareMap.get(IMU.class,"imu");

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
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double notPressedStartTime = 0;
        boolean timing = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ls.setPosition(0.47);
        rs.setPosition(0.53);
        turretAlgorithm.setCenter();
        sleep(300);

        ElapsedTime preTime = new ElapsedTime();
        boolean preState = false;


        ElapsedTime reverseTime = new ElapsedTime();



        // 初始同步位置
        //tl.setPosition(currentPos);
        //tr.setPosition(currentPos);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addData("状态", "IMU偏航角已重置");
        telemetry.update();


        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            ls.setPosition(servoPosition);
            rs.setPosition(1-servoPosition);

            //tr.setPosition(TURRET_CENTER_POSITION);
            //tl.setPosition(TURRET_CENTER_POSITION);


            if (gamepad1.left_bumper){
                imu.resetYaw();
            }

            double distanceCm = -1;
            try {
                distanceCm = distanceSensor.getDistance(
                        org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
            } catch (Exception ignored) {}

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



            PID pid = new PID(P,0,D);
            // 将RPM转换为每秒的编码器刻度数
            double currentVelocityTicks = shooter.getVelocity();
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            double feedForward = F * targetTicksPerSecond;
            double pidPower = pid.update(targetTicksPerSecond-currentVelocityTicks);

            double targetPower = Range.clip(feedForward + pidPower, -1.0, 1.0);
            // 设置电机的目标功率
            shooter.setPower(targetPower);
            shooter2.setPower(targetPower);

            if(gamepad1.a){
                state=false;
                intake.setPower(0.87);
                // blender.setPower(0);//
                TARGET_RPM = 2450;
                reverseState = true;
                reversePower = -0.65;//0.3

                // state=true;
                // intake.setPower(1);
                // blender.setPower(1);
                // TARGET_RPM = 3600;
                // servoPosition = 0.72;
                // blenderPower = 1;

            }
            if(gamepad1.x){
                intake.setPower(0);
                reverseState = false;
                // timeState = true;
                // reverseTime.reset();
                // reversemillitime = 500;
                // reversePower = -0.6;

                servoPosition = 0.644;
                TARGET_RPM = 3200;//3000
            }


            if(gamepad1.right_trigger>0.34){
                preState = true;
                preTime.reset();
                forwardmillitime = 200;
            }
            if(preTime.milliseconds()<forwardmillitime && preState){
                blender.setPower(0.2);
            } else{
                blender.setPower(0);
            }

            if(reverseTime.milliseconds()<reversemillitime && timeState){
                blender.setPower(reversePower);
            }else if(reverseState){
                blender.setPower(reversePower);//0.59
            }else{
                blender.setPower(0);
            }



            if(gamepad1.b){
                intake.setPower(-0.8);
                blender.setPower(-0.55);
                shooter.setPower(0);
                shooter2.setPower(0);
            }



            if(gamepad1.dpad_down){
                TARGET_RPM = 2450;//三角底部 1
                ErrorRangeStart = 150;
                ErrorRangeEnd = 150;
                servoPosition = 0.56;
                normalState = true;
                check = true;
            }
            if(gamepad1.dpad_left){
                TARGET_RPM = 3200;//三角腰 2   3000
                ErrorRangeStart = 1000;
                ErrorRangeEnd = 1000;
                servoPosition = 0.730;//0.634
                normalState = true;
                check = true;
            }
            if(gamepad1.dpad_up){
                TARGET_RPM = 3600;//三角顶点4160 4176 3
                ErrorRangeStart = 200;
                ErrorRangeEnd = 300;
                servoPosition = 0.77;
                normalState = false;
                blenderPower = 1;//0.9
                check = true;
            }
            if(gamepad1.dpad_right){
                TARGET_RPM = 4350;//超远 4
                ErrorRangeStart = 50;
                ErrorRangeEnd = 50;
                servoPosition = 0.85;
                normalState = false;
                blenderPower = 0.85;
                check = true;
            }


            currentPos = Math.max(0.1, Math.min(currentPos, 0.9));

            // --- 同步输出 (这是双舵机的关键) ---
            //tl.setPosition(currentPos);
            //tr.setPosition(currentPos);


            double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
            boolean velocitycheck = false;
            if((TARGET_RPM + ErrorRangeStart)>currentRPM && currentRPM>(TARGET_RPM - ErrorRangeEnd)) {
                velocitycheck = true;
            }
            else {
                velocitycheck = false;
            }

            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState) {
                if (check || velocitycheck) {
                    state = true;}
            }

            if (state && normalState){
                // block.setPosition(1);
                intake.setPower(1);
                blender.setPower(1);
            }
            if (state && !normalState) {
                // block.setPosition(1);
                intake.setPower(0.8);//0.6
                blender.setPower(blenderPower);//0.59
            }
            lastYState = currentYState;

            boolean isPressed = !(distanceCm<4.7);
            // boolean isPressed = touch.digitalRead();

            if (isPressed) {
                // 按下 → 灯灭 + 重置计时
                headlight.setPosition(0);
                timing = false;
            } else {
                // 没按下
                if (!timing) {
                    // 刚进入“没按下”状态 → 启动计时
                    timing = true;
                    notPressedStartTime = runtime.seconds();
                }

                // 如果保持没按下超过 1 秒 → 亮灯
                if (runtime.seconds() - notPressedStartTime >= 0.54) {
                    headlight.setPosition(1);
                }
            }

            turretAlgorithm.update();




            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", TARGET_RPM);
            telemetry.addData("目标 Power", targetPower);
            telemetry.addData("当前 RPM", "%.2f", currentRPM);
            telemetry.addData("Distance (cm)", "%.1f", distanceCm);
            telemetry.addData("Pressed", isPressed);
            telemetry.addData("Light", isPressed ? "OFF" : "waiting/ON");
            telemetry.update();

        }
    }


}

