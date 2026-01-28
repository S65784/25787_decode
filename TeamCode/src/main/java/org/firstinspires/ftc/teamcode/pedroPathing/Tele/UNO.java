package org.firstinspires.ftc.teamcode.pedroPathing.Tele;



import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Algorithm;
import org.firstinspires.ftc.teamcode.pedroPathing.TurretAlgorithm;


import com.bylazar.telemetry.PanelsTelemetry;



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
    private Algorithm algorithm;
    private TurretAlgorithm turretAlgorithm;
    private TelemetryManager telemetryManager;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        algorithm = new Algorithm(hardwareMap);
        turretAlgorithm = new TurretAlgorithm(hardwareMap,telemetry,Algorithm.Alliance.BLUE);
        Algorithm.shootMode4.setServos();
//        Algorithm.ls.setPosition(0.45);
//        Algorithm.rs.setPosition(1-0.45);

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
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
//            if (gamepad2.a) {
//                Algorithm.shootTime(1700, Algorithm.ERROR_RANGE_YI, true, true,2500);
//
//            }

//            if (gamepad1.a) Algorithm.draw();
////                Algorithm.drawTime(3000);
//            if (gamepad2.a) Algorithm.shootMode2.shootTime(true, true, 2000);
//
//
////            Algorithm.shootMode1.shoot(gamepad1.dpad_down);
////            Algorithm.shootMode2.shoot(gamepad1.dpad_left);
////            Algorithm.shootMode3.shoot(gamepad1.dpad_right);
////            Algorithm.shootMode4.shoot(gamepad1.dpad_up);
//             if(gamepad1.dpad_down) Algorithm.shootMode1.shootCheckOnceTime(1000);
//             if(gamepad1.dpad_left) Algorithm.shootMode2.shootCheckOnceTime(2000);
//             if(gamepad1.dpad_right)   Algorithm.shootMode3.shootCheckOnceTime(3000);
//             if(gamepad1.dpad_up)   Algorithm.shootMode4.shootCheckOnceTime(500);

//            if(gamepad1.dpad_down) Algorithm.shootMode1.setServos();
//            if(gamepad1.dpad_left) Algorithm.shootMode2.setServos();
//            if(gamepad1.dpad_right)   Algorithm.shootMode3.setServos();
//            if(gamepad1.dpad_up)   Algorithm.shootMode4.setServos();

           // Algorithm.shootMode1.setServos();
//            Algorithm.ls.setPosition(0.55);
//            Algorithm.rs.setPosition(1-0.55);
            //Algorithm.shooterR.setPower(0.5);

//            if(Algorithm.intakeState==true) Algorithm.draw();




            //if (gamepad1.x) {

//            if (Algorithm.flag(gamepad1.x)) {
//                Algorithm.stopShoot();
//            }

            //boolean yState = Algorithm.flag(gamepad1.y);

//            int mode = -1;
//            if (gamepad1.dpad_down) mode = 1;
//            else if (gamepad1.dpad_left) mode = 2;
//            else if(gamepad1.dpad_up) mode = 3;
//            else if(gamepad1.dpad_right) mode = 4;


            turretAlgorithm.update();




//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//
//            telemetry.addData("Status", "Intake Time: " + Algorithm.drawTimer.toString());
//
//            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Status", "Running");
//            telemetry.addData("目标 RPM", Algorithm.targetRPM);
//            telemetry.addData("当前 RPM", "%.2f", Algorithm.getCurrentRPM());
//            telemetry.addData("test", Algorithm.test);
//            telemetry.addData("intakeState", Algorithm.flag(gamepad1.x));
//            telemetry.addData("YState", Algorithm.flag(gamepad1.y));
            telemetry.update();

//            telemetryManager.addData("current RPM", Algorithm.getCurrentRPM());
//            telemetryManager.update(telemetry);
//            panelsTelemetry.addLine("extra1:${t} extra2:${t*t} extra3:${sqrt(t)}");
//            panelsTelemetry.update(telemetry);

        }
    }
}