package org.firstinspires.ftc.teamcode.pedroPathing.Tele;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

public class UNO extends LinearOpMode {
    private Algorithm Algorihthm;

    private ElapsedTime runtime = new ElapsedTime();
    public static final double MOTOR_TICK_COUNT = 28;
    //public static int TARGET_RPM = 0;
    public static int ERROR_RANGE = 0;


    @Override
    public void runOpMode() {
        Algorihthm = new Algorithm(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Algorithm.servoControl();

        runtime.reset();
        Algorithm.imu.resetYaw();

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

            if (gamepad1.left_bumper) {
                Algorithm.imu.resetYaw();
            }

            //double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            //Algorithm.shooter.setVelocity(targetTicksPerSecond);

            if (gamepad1.a) {
                Algorithm.draw();
            }

            if (Algorithm.flag(gamepad1.x)) {
                Algorithm.stopShoot();
            }

            boolean yState = Algorithm.flag(gamepad1.y);

//            Algorithm.shoot(Algorithm.TARGET_RPM_YI, Algorithm.ERROR_RANGE_YI, Algorithm.flag(gamepad1.dpad_down),yState);
//            Algorithm.shoot(Algorithm.TARGET_RPM_ER, Algorithm.ERROR_RANGE_ER, Algorithm.flag(gamepad1.dpad_left),yState);
//            Algorithm.shoot(Algorithm.TARGET_RPM_SAN, Algorithm.ERROR_RANGE_SAN, Algorithm.flag(gamepad1.dpad_up),yState);
//            Algorithm.shoot(Algorithm.TARGET_RPM_SI, Algorithm.ERROR_RANGE_SI, Algorithm.flag(gamepad1.dpad_right),yState);
            int mode = -1;
            if (gamepad1.dpad_down) {
                mode = 1;
            }else if (gamepad1.dpad_left) {
                mode = 2;
            }else if(gamepad1.dpad_up) {
                mode = 3;
            }else if(gamepad1.dpad_right) {
                mode = 4;
            }
            
            Algorithm.shootMode(mode,yState);

            
            
//            boolean currentYState = gamepad1.y;
//            if (currentYState && !Algorithm.lastYState) {
//                Algorithm.state = true;
//            }
//            Algorithm.lastYState = currentYState;

//            if (gamepad1.dpad_down) {
//                Algorithm.block.setPosition(0.35);
//            }
//            if (gamepad1.dpad_up) {
//                Algorithm.block.setPosition(0.9);
//            }

                telemetry.addData("Status", "Run Time: " + runtime.toString());

            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("目标 RPM", Algorithm.targetRPM);
            telemetry.addData("当前 RPM", "%.2f", Algorithm.getCurrentRPM());
            telemetry.addData("test", Algorithm.test);
            telemetry.addData("intakeState", Algorithm.flag(gamepad1.x));
            telemetry.addData("YState", Algorithm.flag(gamepad1.y));


            telemetry.update();

        }
    }
}