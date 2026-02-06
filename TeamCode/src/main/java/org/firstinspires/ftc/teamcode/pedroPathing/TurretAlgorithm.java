package org.firstinspires.ftc.teamcode.pedroPathing;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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

import java.util.List;


public class TurretAlgorithm {
    private Algorithm.Alliance alliance;
    public    Servo servo1, servo2;
    public DcMotorEx Encoder;
    //public GoBildaPinpointDriver ppt;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public TurretAlgorithm(HardwareMap hardwareMap,Telemetry telemetry ,Algorithm.Alliance alliance,Follower follower,Gamepad gamepad){
        servo1 = hardwareMap.get(Servo.class,"tr");
        servo2 = hardwareMap.get(Servo.class,"tl");
        Encoder = Algorithm.shooter2;
        //ppt = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.follower = follower;


        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        this.alliance = alliance;
        this.telemetry = telemetry;

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        //ppt.resetPosAndIMU();

        gamepad2 = gamepad;
    }
    public TurretAlgorithm(HardwareMap hardwareMap,Telemetry telemetry ,Algorithm.Alliance alliance,Follower follower){
        this(hardwareMap,telemetry,alliance,follower,null);
    }

    public final double FIELD_RANGE = 1750;
    private final double TURRET_GEAR_RATIO = (20.0/96)*(120.0/25);
    private final double SERVO_RANGE = 180.0/130*255;
    private final double TURRET_ROBOT_OFFSET = 50;
    private final double TURRET_CENTER_POSITION = 0.9;
    private final double KALMAN_GAIN = 0.03;
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
    private Gamepad gamepad2;
    private boolean usingCamera;
    private boolean allowCamera = true;
    private boolean allowCameraTime = true;
    private boolean foundTarget = false;
    public boolean autoResetYaw = true;
    public boolean isHardResetYaw = false;
    public boolean isLockCenter = false;
    public double timeThreshold = 1000;
    private double tx;
    private boolean useFar = false;
    private boolean useManually = false;

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
            if(allowCameraTime){
                cameraTime.reset();
            }
        }
    }
    public void updateTx(){
        if(limelightResult!=null && limelightResult.isValid()){
            tx = limelightResult.getTx();
        }
    }

    private final double P=0.001,I=0.00001,D=0.000;
    public PID pid = new PID(P,I,D);
//    public double updatePID(){
//        if()
//        targetLocalHeading = currentDegrees - targetTurretHeading;
//        double error = normalizeAngle(targetLocalHeading-currentDegrees);
//        return pid.update(error);
//    }



    /**
     * 更新PID逻辑：
     * 1. 优先尝试识别 ID 为 21 的 AprilTag。
     * 2. 如果识别到 ID 21，使用其 TX (水平偏差) 进行视觉闭环控制。
     * 3. 如果没识别到 ID 21 (即使有其他Tag)，回退到使用里程计/场坐标控制。
     */
    public double updatePID() {
        boolean foundID = false;
        double txError = 0;

        // 1. 检查 Limelight 结果是否有效
        if (limelightResult != null && limelightResult.isValid()) {
            // 获取所有识别到的标签列表
            List<LLResultTypes.FiducialResult> fiducials = limelightResult.getFiducialResults();

            if (fiducials != null) {
                // 2. 遍历列表，专门寻找 ID 21
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (tag.getFiducialId() == alliance.getID()) {
                        txError = tx; // 获取水平角度偏差
                        foundID = true;
                        break; // 找到目标后，停止遍历
                    }
                }
            }
        }

        if (foundID) {
            // === 视觉闭环模式 (ID 21) ===

            // 更新 targetLocalHeading。这一步是为了让 setServoPosition 中的前馈计算(Feedforward)保持连贯。
            // 逻辑：目标位置(绝对) = 当前位置 + 偏差
            // 如果发现转塔往反方向修正，请将下面的 + txError 改为 - txError
            targetLocalHeading = normalizeAngle(currentDegrees + txError);

            // 将 tx 直接作为 PID 的误差输入
            // tx > 0 表示目标在右侧。通常 PID 输出正值会驱动转塔向右转。
            return pid.update(txError);

        } else {
            // === 里程计/场坐标模式 (无 ID 21) ===
            // 传统的场坐标计算逻辑
            targetLocalHeading = targetTurretHeading - currentRobotHeading;
            double error = normalizeAngle(targetLocalHeading - currentDegrees);

            return pid.update(error);
        }
    }


    public void setTargetTurretHeading(){
        if(useFar){
            targetTurretHeading = alliance.getFarTurretHeading();
        }else {
            if(alliance == Algorithm.Alliance.RED){targetTurretHeading = normalizeAngle(-Point.getAngleFromPoints(getTurretPoint(), alliance.getPoint()) + 90-5);}
            else{targetTurretHeading = normalizeAngle(-Point.getAngleFromPoints(alliance.getPoint(), getTurretPoint()) - 90);}
        }
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
        currentTicks = -Encoder.getCurrentPosition();
    }
    public void setDegrees(){
        setTicks();
        currentDegrees = (currentTicks / TICKS_PER_REV) * (20.0/96.0) * 360.0;
    }

    public void useFar(){
        useFar = true;
    }

    public void useAllTime(){
        useFar = false;
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
    public void unlock(){
        isLockCenter = false;
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
    }

    public void allowCamera(){
        allowCamera = true;
    }
    public void banCamera(){
        allowCamera = false;
    }

    public void chanceAlliance(){
        if(alliance == Algorithm.Alliance.RED)alliance = Algorithm.Alliance.BLUE;
        else alliance = Algorithm.Alliance.RED;

    }
    public void init(){
        setCenter();
        Algorithm.sleep(300);
        Encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setManually(){
        useManually = true;
    }
    public void stopManually(){
        useManually = false;
    }

    public void useManually(){
        if(gamepad2.dpad_down){
            servo1.setPosition(1-0.1);
            servo2.setPosition(1-0.1);

        }
        if(gamepad2.dpad_left){
            servo1.setPosition(1-0.36);
            servo2.setPosition(1-0.36);

        }
        if(gamepad2.dpad_up){
            servo1.setPosition(1-0.63);
            servo2.setPosition(1-0.63);
        }
        if(gamepad2.dpad_right){
            servo1.setPosition(1-0.93);
            servo2.setPosition(1-0.93);
        }
    }

    public void updateTelemetry(){
        telemetry.addData("using camera",usingCamera);
        telemetry.addData("use far",useFar);
        telemetry.addData("allow camera", allowCamera);
        telemetry.addData("allow by time",allowCameraTime);
        telemetry.addData("x offset",xTotalOffset);
        telemetry.addData("y offset",yTotalOffset);
        telemetry.addData("yaw offset",yawTotalOffset);
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
        updateTx();
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
        }else if(useManually && gamepad2!=null){
            useManually();
        }
        else{
            setCenter();
        }
    }


    public static double normalizeAngle(double  angle){
        return AngleUnit.normalizeDegrees(angle);
    }

}