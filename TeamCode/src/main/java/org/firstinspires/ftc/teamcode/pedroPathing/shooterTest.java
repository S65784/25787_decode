/*
Copyright 2026 FIRST Tech Challenge Team 25787

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

public class shooterTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotor blender = null;
    private DcMotorEx shooter = null;
    private Servo ls = null;
    private Servo rs = null;

    //private Servo block = null;
    private boolean lastYState = false;
    private boolean state = false;

    ElapsedTime shooterDecelarateTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "Blender");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        ls  = hardwareMap.get(Servo.class, "ls");
        rs  = hardwareMap.get(Servo.class, "rs");

        //block = hardwareMap.get(Servo.class, "Block");

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
        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ls.setPosition(0.4);
            rs.setPosition(0.6);

            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

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


            if(gamepad1.a){
                intake.setPower(0.8);
                //blender.setPower(0.55);
                shooter.setPower(0);
            }

            if(gamepad1.b){
                intake.setPower(-0.8);
                //blender.setPower(-0.55);
                shooter.setPower(0);
            }
            if(gamepad1.square){
                shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sleep(300);
            }

            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState && !state) {
                intake.setPower(0);
                blender.setPower(-0.5);
                //sleep(50);
                shooter.setPower(0.55);
                sleep(150);
                blender.setPower(0);
                state = true;
            } else if (currentYState && !lastYState && state) {
                intake.setPower(1);
                blender.setPower(0.55);
                //shooter.setPower(0.55);
                ShooterShoot(6000*0.45, 1,0.45);
                //sleep(100);
                state = false;
            }

            lastYState = currentYState;

            if(gamepad1.x){
                shooterDecelarateTime.reset();
            }
            if(shooterDecelarateTime.seconds()<1){
                shooter.setPower(-1);
            }else if(shooterDecelarateTime.seconds()<1.1){
                shooter.setPower(0);
            }

//            if(gamepad1.dpad_down){//低
//                block.setPosition(0.23);
//            }
//            if(gamepad1.dpad_up){
//                block.setPosition(0.5);
//            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

    private void ShooterShoot(double targetSpeed,double Kp, double originalPower){
        double speed = shooter.getVelocity()/28;
        double error = speed-targetSpeed;
        shooter.setPower(Kp*error+originalPower);
        if(error<1&&error>-1){
            gamepad1.rumble(200);
        }
    }
}
