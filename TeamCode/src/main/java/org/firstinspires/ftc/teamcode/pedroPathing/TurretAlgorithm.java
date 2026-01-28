package org.firstinspires.ftc.teamcode.pedroPathing;



import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.GoBildaPinpointDriver;


public class TurretAlgorithm {
    public Algorithm.Alliance alliance;
    public  Servo servo1, servo2;
    public GoBildaPinpointDriver ppt;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public TurretAlgorithm(HardwareMap hardwareMap,Telemetry telemetry ,Algorithm.Alliance alliance){
        servo1 = hardwareMap.get(Servo.class,"tr");
        servo2 = hardwareMap.get(Servo.class,"tl");
        ppt = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


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
    public final double TURRET_CENTER_POSITION = 0.34;
    public final double KALMAN_GAIN = 0.3;
    public double currentRobotHeading;
    public double targetTurretHeading;
    public double cameraTurretHeading;
    public double calcTurretHeading;
    public double turretLocalHeading = 0;
    public double targetLocalHeading;

    public double xTotalOffset = 0;
    public double yTotalOffset = 0;
    public double xPartialOffset;
    public double yPartialOffset;

    public double servoPosition;
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
        currentRobotHeading = ppt.getPosition().getHeading(AngleUnit.DEGREES);
        calcTurretHeading = (servo1.getPosition()-TURRET_CENTER_POSITION)*SERVO_RANGE*TURRET_GEAR_RATIO+currentRobotHeading;
usingCamera = false;
    }

    public void updatePointWithCamera(){
        if(limelightResult!=null && limelightResult.isValid()){//&& Algorithm.shootState
            cameraPose = limelightResult.getBotpose();
            cameraTurretPoint = new Point(cameraPose.getPosition().x*1000, cameraPose.getPosition().y*1000);
            cameraTurretHeading = cameraPose.getOrientation().getYaw();
            usingCamera = true;
        }
    }

    private final double P=0,I=0,D=0;
    public PID pid = new PID(P,I,D);
    public double updatePID(){
        targetLocalHeading = targetTurretHeading - currentRobotHeading;
        double currentServoAngle = (servo1.getPosition() - TURRET_CENTER_POSITION) * SERVO_RANGE*TURRET_GEAR_RATIO;
    double error = normalizeAngle(targetLocalHeading-currentServoAngle);
        return pid.update(error);
    }

    public void setTargetTurretHeading(){
        targetTurretHeading = Point.getAngleFromPoints(getTurretPoint(),alliance.getPoint());
    }
    public void setServoPosition(){
        //may need to be changed
        servoPosition = TURRET_CENTER_POSITION + targetLocalHeading/(SERVO_RANGE*TURRET_GEAR_RATIO)-pidOut;
        servoPosition = Range.clip(servoPosition,0,1);
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


    public void adjustPoint(){
        setPointFromChassis(rawRobotPoint);
        if(usingCamera) {
            xTotalOffset = cameraTurretPoint.x - calcTurretPoint.x;
            yTotalOffset = cameraTurretPoint.y - calcTurretPoint.y;
        }
        updateOffset();
        adjustedRobotPoint = new Point(calcTurretPoint.x+xPartialOffset,calcTurretPoint.y+yPartialOffset);
    }

    public void updateOffset(){
        xPartialOffset = xPartialOffset*(1-KALMAN_GAIN)+xTotalOffset*KALMAN_GAIN;
        yPartialOffset = yPartialOffset*(1-KALMAN_GAIN)+yTotalOffset*KALMAN_GAIN;
    }


    public Point getTurretPoint(){
        return adjustedRobotPoint;
    }

    public void setPointFromChassis(Point robotPoint){
        double xOffset = Math.cos(Math.toRadians(currentRobotHeading))*TURRET_ROBOT_OFFSET;
        double yOffset = Math.sin(Math.toRadians(currentRobotHeading))*TURRET_ROBOT_OFFSET;
        calcTurretPoint = new Point(robotPoint.x+xOffset,robotPoint.y+yOffset);
    }


    public void update(){
        limelightResult = limelight.getLatestResult();
        updateCurrentPointWithoutCamera();
        updatePointWithCamera();
        adjustPoint();
        turretLocalHeading = normalizeAngle(calcTurretHeading-currentRobotHeading);
        setTargetTurretHeading();
        pidOut = updatePID();
        setServoPosition();
        telemetry.addData("using camera",usingCamera);
    }

    public static double normalizeAngle(double Angle){
        while (Angle>180) Angle-=360;
        while (Angle<=-180) Angle+=360;
        return Angle;
    }
}