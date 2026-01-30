package org.firstinspires.ftc.teamcode.pedroPathing;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.GoBildaPinpointDriver;


public class TurretAlgorithm {
    public Algorithm.Alliance alliance;
    public  Servo servo1, servo2;
    public DcMotorEx Encoder;
    public GoBildaPinpointDriver ppt;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public TurretAlgorithm(HardwareMap hardwareMap,Telemetry telemetry ,Algorithm.Alliance alliance){
        servo1 = hardwareMap.get(Servo.class,"tr");
        servo2 = hardwareMap.get(Servo.class,"tl");
        Encoder = hardwareMap.get(DcMotorEx.class,"ShooterR");
        ppt = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        this.alliance = alliance;
        this.telemetry = telemetry;

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        ppt.resetPosAndIMU();
    }

    public final double TURRET_GEAR_RATIO = (20.0/96)*(120.0/25);
    public final double SERVO_RANGE = 180.0/130*255;
    public final double TURRET_ROBOT_OFFSET = 50;
    public final double TURRET_CENTER_POSITION = 0.9;
    public final double KALMAN_GAIN = 0.03;
    public double currentRobotHeading;
    public double targetTurretHeading;
    public double cameraTurretHeading;
    public double calcTurretHeading;
    public double turretLocalHeading = 0;
    public double targetLocalHeading;
    public double rawRobotHeading;
    public double calcRobotHeading;
    public double currentServoAngle;
    public double adjustedRobotHeading;

    public double xTotalOffset = 0;
    public double yTotalOffset = 0;
    public double yawTotalOffset = 0;
    public double xPartialOffset;
    public double yPartialOffset;
    public double yawPartialOffset;
    public double yawLastOffset = 0;

    public double servoPosition;

    final double TICKS_PER_REV = 8192.0;

    public double pidOut;
    public Pose3D cameraPose;
    public Point rawRobotPoint;
    public Point adjustedRobotPoint = new Point(0,0);
    public Point cameraTurretPoint = new Point(0,0);
    public Point calcTurretPoint;
    public LLResult limelightResult;
    private boolean usingCamera;


    public void updateCurrentPointWithoutCamera(){
        ppt.update();
        rawRobotPoint = new Point(ppt.getPosition().getX(DistanceUnit.MM),ppt.getPosition().getY(DistanceUnit.MM));
        rawRobotHeading = normalizeAngle(ppt.getPosition().getHeading(AngleUnit.DEGREES));
        calcTurretHeading = normalizeAngle((servo1.getPosition()-TURRET_CENTER_POSITION)*SERVO_RANGE*TURRET_GEAR_RATIO+currentRobotHeading);
        usingCamera = false;
    }

    public void updatePointWithCamera(){
        if(limelightResult!=null && limelightResult.isValid()){//&& Algorithm.shootState
            cameraPose = limelightResult.getBotpose();
            cameraTurretPoint = new Point(-cameraPose.getPosition().x*1000, cameraPose.getPosition().y*1000);
            cameraTurretHeading = normalizeAngle(cameraPose.getOrientation().getYaw()-90);
            usingCamera = true;
            //updatePpt();
        }
    }

    private final double P=0,I=0,D=0;
    public PID pid = new PID(P,I,D);
    public double updatePID(){
        targetLocalHeading = normalizeAngle(targetTurretHeading - currentRobotHeading);
    double error = normalizeAngle(targetLocalHeading-currentServoAngle);
        return pid.update(error);
    }

    public void setTargetTurretHeading(){
        targetTurretHeading = normalizeAngle(Point.getAngleFromPoints(getTurretPoint(),alliance.getPoint())-180);
    }
    public void setServoPosition(){
        //may need to be changed
        servoPosition = TURRET_CENTER_POSITION + targetLocalHeading/(SERVO_RANGE*TURRET_GEAR_RATIO)-pidOut;
        servoPosition = Range.clip(servoPosition,0.08,0.92);
        servo1.setPosition(servoPosition);
        servo2.setPosition(servoPosition);
    }

//    public void updatePosition(){
//        if(limelightResult!=null && limelightResult.isValid()){//&& Algorithm.shootState
//            turretPosition = limelightResult.getBotpose();
//            turretPoint = new Point(turretPosition.getPosition().x*1000, turretPosition.getPosition().y*1000);
//            currentTurretHeading = turretPosition.getOrientation().getYaw();
//            usingCamera = true;
//        }else{
//            updateCurrentPointWithoutCamera();
//            setPointFromChassis(robotPoint);
//            usingCamera = false;
//        }
//    }

    int currentTicks;
    double currentDegrees;
    public double getTicks(){
        currentTicks = Encoder.getCurrentPosition();
        return currentTicks;
    }
    public double getDegrees(){
        currentDegrees = (currentTicks / TICKS_PER_REV) * 360.0;
        return currentDegrees;
    }


    public void adjustHeading(){
        if(usingCamera){
            yawTotalOffset = normalizeAngle((cameraTurretHeading-currentServoAngle) - rawRobotHeading);
        }
//        if(Math.abs(normalizeAngle(yawPartialOffset-yawLastOffset))>45){
//            return;
//        }
        currentRobotHeading = normalizeAngle(rawRobotHeading+yawPartialOffset);
        yawPartialOffset = yawLastOffset;
    }

    public void adjustPoint(){
        setPointFromChassis(rawRobotPoint);
        if(usingCamera) {
            xTotalOffset = cameraTurretPoint.x - calcTurretPoint.x;
            yTotalOffset = cameraTurretPoint.y - calcTurretPoint.y;
        }
        adjustedRobotPoint = new Point(calcTurretPoint.x+xPartialOffset,calcTurretPoint.y+yPartialOffset);
    }

    public void updateOffset(){
        xPartialOffset = xPartialOffset*(1-KALMAN_GAIN)+xTotalOffset*KALMAN_GAIN;
        yPartialOffset = yPartialOffset*(1-KALMAN_GAIN)+yTotalOffset*KALMAN_GAIN;
        yawPartialOffset = yawPartialOffset*(1-KALMAN_GAIN)+yawTotalOffset*KALMAN_GAIN;
    }

//    public void updatePpt(){
//        ppt.setHeading(currentRobotHeading,AngleUnit.DEGREES);
//    }


    public Point getTurretPoint(){
        return adjustedRobotPoint;
    }

    public void setPointFromChassis(Point robotPoint){
        double xOffset = Math.cos(Math.toRadians(currentRobotHeading))*TURRET_ROBOT_OFFSET;
        double yOffset = Math.sin(Math.toRadians(currentRobotHeading))*TURRET_ROBOT_OFFSET;
        calcTurretPoint = new Point(robotPoint.x+xOffset,robotPoint.y+yOffset);
    }

    public void lockCenter(){
        servo1.setPosition(TURRET_CENTER_POSITION);
        servo2.setPosition(TURRET_CENTER_POSITION);
    }

    public void updateTelemetry(){
        telemetry.addData("using camera",usingCamera);
        telemetry.addData("x offset",xTotalOffset);
        telemetry.addData("y offset",yTotalOffset);
        telemetry.addData("camera point x",cameraTurretPoint.x);
        telemetry.addData("camera point y",cameraTurretPoint.y);
        telemetry.addData("robot point x",adjustedRobotPoint.x);
        telemetry.addData("robot point y",adjustedRobotPoint.y);
        telemetry.addData("calc heading",calcTurretHeading);
        telemetry.addData("target heading",targetTurretHeading);
        telemetry.addData("robot heading",currentRobotHeading);
        telemetry.addData("servo position",servoPosition);
        telemetry.addData("ticks",currentTicks);
        telemetry.addData("degrees",currentDegrees);
    }

    public void update(){
        currentServoAngle = (servo1.getPosition() - TURRET_CENTER_POSITION) * SERVO_RANGE*TURRET_GEAR_RATIO;
        limelightResult = limelight.getLatestResult();
        updateCurrentPointWithoutCamera();
        updatePointWithCamera();
        adjustPoint();
        adjustHeading();
        updateOffset();
        turretLocalHeading = normalizeAngle(calcTurretHeading-currentRobotHeading);
        setTargetTurretHeading();
        pidOut = updatePID();
        setServoPosition();
        getTicks();
        getDegrees();
        updateTelemetry();
    }

    public static double normalizeAngle(double Angle){
        while (Angle>180) Angle-=360;
        while (Angle<=-180) Angle+=360;
        return Angle;
    }

}