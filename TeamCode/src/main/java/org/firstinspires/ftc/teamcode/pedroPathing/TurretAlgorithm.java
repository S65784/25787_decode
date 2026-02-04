package org.firstinspires.ftc.teamcode.pedroPathing;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.GoBildaPinpointDriver;


public class TurretAlgorithm {
    private Algorithm.Alliance alliance;
    public    Servo servo1, servo2;
    public DcMotorEx Encoder;
    //public GoBildaPinpointDriver ppt;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public TurretAlgorithm(HardwareMap hardwareMap,Telemetry telemetry ,Algorithm.Alliance alliance,Follower follower){
        servo1 = hardwareMap.get(Servo.class,"tr");
        servo2 = hardwareMap.get(Servo.class,"tl");
        Encoder = hardwareMap.get(DcMotorEx.class,"ShooterR");
        //ppt = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Encoder.setDirection(DcMotorSimple.Direction.FORWARD);
        this.follower = follower;
        Encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        this.alliance = alliance;
        this.telemetry = telemetry;

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        //ppt.resetPosAndIMU();
    }

    public final double FIELD_RANGE = 1750;
    private final double TURRET_GEAR_RATIO = (20.0/96)*(120.0/25);
    private final double SERVO_RANGE = 180.0/130*255;
    private final double TURRET_ROBOT_OFFSET = 50;
    private final double TURRET_CENTER_POSITION = 0.9;
    private final double KALMAN_GAIN = 0.3;
    private double currentRobotHeading;
    public double targetTurretHeading;
    public double cameraTurretHeading;
    public double calcTurretHeading;
    public double turretLocalHeading = 0;
    public double targetLocalHeading;
    public double rawRobotHeading;
    public double calcRobotHeading;
    public double currentServoAngle;
    public double adjustedRobotHeading;
    private double encoderTurretHeading;

    public double xTotalOffset = 0;
    public double yTotalOffset = 0;
    public double yawTotalOffset = 0;
    public double xPartialOffset;
    public double yPartialOffset;
    public double yawPartialOffset;
    public double yawLastOffset = 0;
    public double xRawOffset;
    public double yRawOffset;

    public double servoPosition;

    final double TICKS_PER_REV = 8192.0;

    public double pidOut;
    Follower follower;
    public Pose3D cameraPose;
    public Point rawRobotPoint;
    public Point adjustedRawPoint;
    public Point adjustedRobotPoint = new Point(0,0);
    public Point cameraTurretPoint = new Point(0,0);
    public Point calcTurretPoint;
    public Pose2D robotPose;
    public LLResult limelightResult;
    private boolean usingCamera;
    private boolean allowCamera = true;
    private boolean allowCameraTime = true;
    private boolean foundTarget = false;
    public boolean autoResetYaw = true;
    public boolean isHardResetYaw = false;
    public boolean isLockCenter = false;
    public double timeThreshold = 1000;

    ElapsedTime cameraTime = new ElapsedTime();


    public void updateCurrentPointWithoutCamera(){
        follower.update();
        //ppt.update();
        //robotPose = ppt.getPosition();
        robotPose = new Pose2D(DistanceUnit.INCH,follower.getPose().getX(),follower.getPose().getY(),AngleUnit.RADIANS,follower.getHeading());
        //rawRobotPoint = new Point(-follower.getPose().getY(),follower.getPose().getX());
        rawRobotPoint = new Point(robotPose.getY(DistanceUnit.MM),robotPose.getX(DistanceUnit.MM));// new Point(robotPose.getX(DistanceUnit.MM),robotPose.getY(DistanceUnit.MM))
        rawRobotHeading = normalizeAngle(robotPose.getHeading(AngleUnit.DEGREES));
        //rawRobotHeading = follower.getHeading();
        calcTurretHeading = normalizeAngle((servo1.getPosition()-TURRET_CENTER_POSITION)*SERVO_RANGE*TURRET_GEAR_RATIO+currentRobotHeading);
        adjustRawPoint();
        usingCamera = false;
    }



    public void updatePointWithCamera(){
        if(limelightResult!=null && limelightResult.isValid()){//&& Algorithm.shootState  && cameraTime.milliseconds()>CAMERA_INTERVAL
            cameraPose = limelightResult.getBotpose();
            cameraTurretPoint = new Point(-cameraPose.getPosition().x*1000, cameraPose.getPosition().y*1000);
            cameraTurretHeading = normalizeAngle(cameraPose.getOrientation().getYaw()-90);//-90
            usingCamera = true;
            foundTarget = true;
            //updatePpt();
            cameraTime.reset();
        }
    }

    private final double P=0.00003,I=0,D=0.000;
    public PID pid = new PID(P,I,D);
    public double updatePID(){
        targetLocalHeading = targetTurretHeading - currentRobotHeading;
        double error = normalizeAngle(targetLocalHeading-currentDegrees);
        return pid.update(error);
    }

    public void setTargetTurretHeading(){
        targetTurretHeading = normalizeAngle(-Point.getAngleFromPoints(getTurretPoint(),alliance.getPoint())+90);
    }
    public void setServoPosition(){
        //may need to be changed
        double target = targetLocalHeading;
        if(target-currentDegrees>180){
            target-=360;
        }else if(target-currentDegrees<-180){
            target+=360;
        }
        servoPosition = TURRET_CENTER_POSITION + target/(SERVO_RANGE*TURRET_GEAR_RATIO)+pidOut;//targetLocalHeading
        double servoPositionClip = Range.clip(servoPosition,0,1);
        servo1.setPosition(servoPositionClip);
        servo2.setPosition(servoPositionClip);
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
    public void setTicks(){
        currentTicks = Encoder.getCurrentPosition();
    }
    public void setDegrees(){
        setTicks();
        currentDegrees = (currentTicks / TICKS_PER_REV) * (20.0/96.0) * 360.0;
    }


    public void adjustHeading(){
        if(usingCamera && allowCamera && allowCameraTime && autoResetYaw){
            yawTotalOffset = normalizeAngle((cameraTurretHeading - currentDegrees) - rawRobotHeading);

        }
//        if(Math.abs(normalizeAngle(yawPartialOffset-yawLastOffset))>45){
//            return;
//        }
        currentRobotHeading = normalizeAngle(rawRobotHeading+yawPartialOffset);
        //yawPartialOffset = yawLastOffset;
    }

    public void resetYaw(){
        //yawTotalOffset = -ppt.getPosition().getHeading(AngleUnit.DEGREES);
        autoResetYaw = false;
    }

    public void hardResetYaw(){
        follower.setPose(new Pose(0,0,0));
        isHardResetYaw = true;
    }

    public void lockCenter(){
        isLockCenter = true;
    }


    public void adjustPoint(){
        //if(!isHardResetYaw) {
            setPointFromChassis(adjustedRawPoint);
        //}else{
        //    setPointFromChassis(rawRobotPoint);
        //}
        if(usingCamera && allowCamera && allowCameraTime) {
            xTotalOffset = cameraTurretPoint.x - calcTurretPoint.x;
            yTotalOffset = cameraTurretPoint.y - calcTurretPoint.y;
        }
        adjustedRobotPoint = new Point(Range.clip(calcTurretPoint.x+xPartialOffset,-FIELD_RANGE,FIELD_RANGE),Range.clip(calcTurretPoint.y+yPartialOffset,-FIELD_RANGE,FIELD_RANGE));
    }

    public void adjustRawPoint(){
        xRawOffset = (rawRobotPoint.x*Math.cos(Math.toRadians(yawTotalOffset))-rawRobotPoint.y*Math.sin(Math.toRadians(yawTotalOffset)));
        yRawOffset = (rawRobotPoint.x*Math.sin(Math.toRadians(yawTotalOffset))+rawRobotPoint.y*Math.cos(Math.toRadians(yawTotalOffset)));
        adjustedRawPoint = new Point(xRawOffset,yRawOffset);
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

    public void setCenter(){
        servo1.setPosition(TURRET_CENTER_POSITION);
        servo2.setPosition(TURRET_CENTER_POSITION);
        Encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void allowCamera(){
        allowCamera = true;
    }
    public void banCamera(){
        allowCamera = false;
    }

    public void updateTelemetry(){
        telemetry.addData("using camera",usingCamera);
        telemetry.addData("allow camera", allowCamera);
        telemetry.addData("allow by time",allowCameraTime);
        telemetry.addData("x offset",xTotalOffset);
        telemetry.addData("y offset",yTotalOffset);
        //telemetry.addData("yaw offset",yawTotalOffset);
        telemetry.addData("camera point x",cameraTurretPoint.x);
        telemetry.addData("camera point y",cameraTurretPoint.y);
        telemetry.addData("raw x",rawRobotPoint.x);
        telemetry.addData("raw y",rawRobotPoint.y);
        //telemetry.addData("adjusted raw x",adjustedRawPoint.x);
        //telemetry.addData("adjusted raw y",adjustedRawPoint.y);
        //telemetry.addData("calc turret Point x",calcTurretPoint.x);
        //telemetry.addData("calc turret Point y",calcTurretPoint.y);
        telemetry.addData("robot point x",adjustedRobotPoint.x);
        telemetry.addData("robot point y",adjustedRobotPoint.y);
        //telemetry.addData("camera heading",cameraTurretHeading);
        //telemetry.addData("calc heading",calcTurretHeading);
        telemetry.addData("turret heading",encoderTurretHeading);
        telemetry.addData("target heading",targetTurretHeading);
        telemetry.addData("robot heading",currentRobotHeading);
        telemetry.addData("servo position",servoPosition);
        telemetry.addData("camera heading", cameraTurretHeading);
        //telemetry.addData("ticks",currentTicks);
        telemetry.addData("degrees",currentDegrees);
    }

    public void update(){
        allowCameraTime = !(cameraTime.milliseconds() < timeThreshold);
        currentServoAngle = (servo1.getPosition() - TURRET_CENTER_POSITION) * SERVO_RANGE * TURRET_GEAR_RATIO;
        limelightResult = limelight.getLatestResult();
        updateCurrentPointWithoutCamera();
        updatePointWithCamera();
        adjustHeading();
        adjustPoint();
        updateOffset();
        turretLocalHeading = normalizeAngle(calcTurretHeading - currentRobotHeading);
        encoderTurretHeading = normalizeAngle(currentRobotHeading + currentDegrees);
        setTargetTurretHeading();
        pidOut = updatePID();
        setDegrees();
        updateTelemetry();
        if((foundTarget || !allowCamera) && !isLockCenter) {//foundTarget &&
            setServoPosition();
        }else{
            setCenter();
        }
    }

//    public static double normalizeAngle(double Angle){
//        while (Angle>180) Angle-=360;
//        while (Angle<=-180) Angle+=360;
//        return Angle;
//    }

    public static double normalizeAngle(double  angle){
        return AngleUnit.normalizeDegrees(angle);
    }

}