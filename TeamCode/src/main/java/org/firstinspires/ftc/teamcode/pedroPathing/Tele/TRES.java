package org.firstinspires.ftc.teamcode.pedroPathing.Tele;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.TurretAlgorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.TurretAlgorithm;


import com.bylazar.telemetry.PanelsTelemetry;
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

public class TRES extends LinearOpMode {
    private Algorithm algorithm;
    private TurretAlgorithm turretAlgorithm;
    private TelemetryManager telemetryManager;

    private boolean lastYState = false;
    private boolean state = false;
    public static final double MOTOR_TICK_COUNT = 28;

    public static double P = 0.005, I = 0, D = 0;
    public static double F = 0.0004;
    public static double TARGET_RPM = 0;
    public static double ErrorRangeStart = 200;
    public static double ErrorRangeEnd = 200;
    public static double servoPosition = 0.47;
    public static boolean normalState = true;
    public static double blenderPower = 0;
    public static boolean check = false;

    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        algorithm = new Algorithm(hardwareMap);
        turretAlgorithm = new TurretAlgorithm(hardwareMap,telemetry,Algorithm.Alliance.BLUE);
        //Algorithm.shootMode4.setServos();
//        Algorithm.ls.setPosition(0.45);
//        Algorithm.rs.setPosition(1-0.45);
        turretAlgorithm.lockCenter();
        Algorithm.servoControl();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime shootTime = new ElapsedTime();
        boolean shootState = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        runtime.reset();
        Algorithm.imu.resetYaw();

        while (opModeIsActive()) {
            algorithm.ls.setPosition(servoPosition);
            algorithm.rs.setPosition(1-servoPosition);



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


            PID pid = new PID(P,0,D);
            // 将RPM转换为每秒的编码器刻度数
            double currentVelocityTicks = algorithm.shooter.getVelocity();
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            // 设置电机的目标速度
            double feedForward = F * targetTicksPerSecond;
            double pidPower = pid.update(targetTicksPerSecond-currentVelocityTicks);

            double targetPower = Range.clip(feedForward + pidPower, -1.0, 1.0);
            // 设置电机的目标功率
            algorithm.shooter.setPower(targetPower);
            algorithm.shooter2.setPower(targetPower);

            if(gamepad1.a){
                state=false;
                algorithm.intake.setPower(1);
                algorithm.blender.setPower(0);
                TARGET_RPM = 2450;

                // state=true;
                // intake.setPower(1);
                // blender.setPower(1);
                // TARGET_RPM = 3600;
                // servoPosition = 0.72;
                // blenderPower = 1;

            }


            if(gamepad1.right_trigger>0.34){
                shootState = true;
                shootTime.reset();
            }
            if(shootTime.seconds()<0.2 && shootState){
                algorithm.blender.setPower(0.2);
            }else{
                algorithm.blender.setPower(0);
            }



            if(gamepad1.b){
                algorithm.intake.setPower(-0.8);
                algorithm.blender.setPower(-0.55);
                algorithm.shooter.setPower(0);
                algorithm.shooter2.setPower(0);
            }

            if(gamepad1.x){
                algorithm.intake.setPower(0);
                // blender.setPower(0);
                servoPosition = 0.6;
                TARGET_RPM = 2450;
            }

            if(gamepad1.dpad_down){
                TARGET_RPM = 2450;//三角底部 1
                ErrorRangeStart = 150;
                ErrorRangeEnd = 150;
                servoPosition = 0.6;
                normalState = true;
                check = true;
            }
            if(gamepad1.dpad_left){
                TARGET_RPM = 3000;//三角腰 2
                ErrorRangeStart = 1000;
                ErrorRangeEnd = 1000;
                servoPosition = 0.634;//0.62
                normalState = true;
                check = true;
            }
            if(gamepad1.dpad_up){
                TARGET_RPM = 3920;//三角顶点4160 4176 3
                ErrorRangeStart = 200;
                ErrorRangeEnd = 300;
                servoPosition = 0.76;
                normalState = false;
                blenderPower = 1;//0.9
                check = true;
            }
            if(gamepad1.dpad_right){
                TARGET_RPM = 5000;//超远 4
                ErrorRangeStart = 100;
                ErrorRangeEnd = 200;
                servoPosition = 0.57;
                normalState = false;
                blenderPower = 0.45;
                check = true;
            }


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

                algorithm.intake.setPower(1);
                algorithm.blender.setPower(1);
            }
            if (state && !normalState) {

                algorithm.intake.setPower(0.8);//0.6
                algorithm.blender.setPower(blenderPower);//0.59
            }
            lastYState = currentYState;





            if (gamepad1.left_bumper) {
                Algorithm.imu.resetYaw();
            }





            turretAlgorithm.update();




//
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", TARGET_RPM);
            telemetry.addData("当前 RPM", "%.2f", currentRPM);

            telemetry.update();

//
        }
    }
}