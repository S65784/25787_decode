package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.EchoLapse.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope.AprilTagLocalizer;

public class TurretAlgorithm {
    public AprilTagLocalizer aprilTagLocalizer;
    public Algorithm.Alliance alliance;
    public static Servo servo1, servo2;
    public static GoBildaPinpointDriver ppt;
    public TurretAlgorithm(HardwareMap hardwareMap,AprilTagLocalizer aprilTagLocalizer ,Algorithm.Alliance alliance){
        servo1 = hardwareMap.get(Servo.class,"servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        ppt = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        this.alliance = alliance;
        this.aprilTagLocalizer = aprilTagLocalizer;

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        ppt.resetPosAndIMU();
    }

    public final double TURRET_GEAR_RATIO = (20.0/96)*(120.0/20);
    public final double SERVO_RANGE = 180.0/130*255;
    public final double TURRET_ROBOT_OFFSET = -1;
    public double currentRobotHeading;
    public double targetTurretHeading;
    public double currentTurretHeading;
    public double turretLocalHeading;

    public double servoPosition;
    public double pidOut;
    public Pose2D turretPosition;
    public Pose2D robotPosition;
    public Point robotPoint;
    public Point turretPoint;


    public void updateCurrentPointWithoutCamera(){
        robotPoint = new Point(ppt.getPosition().getX(DistanceUnit.MM),ppt.getPosition().getY(DistanceUnit.MM));

        currentTurretHeading = servo1.getPosition()*SERVO_RANGE*TURRET_GEAR_RATIO;
    }

    private double P,I,D;
    public PID pid = new PID(P,I,D);
    public double updatePID(){
        double error = currentTurretHeading - targetTurretHeading;
        if (error>180) error-=360;
        if(error<=-180) error+=360;
        return pid.update(error);
    }


    public void setTargetTurretHeading(){
        targetTurretHeading = Point.getAngleFromPoints(getTurretPoint(),alliance.getPoint());
    }
    public void setServoPosition(){
        //may need to be changed
        servoPosition = 0.5 + targetTurretHeading/(SERVO_RANGE*TURRET_GEAR_RATIO)+pidOut;
        servoPosition = Range.clip(servoPosition,0,1);
        servo1.setPosition(servoPosition);
        servo2.setPosition(servoPosition);
    }

    public void updatePosition(){
        if(aprilTagLocalizer.getRobotPose() != null ){//&& Algorithm.shootState
            turretPosition = aprilTagLocalizer.getRobotPose();
            turretPoint = new Point(turretPosition.getX(DistanceUnit.MM),turretPosition.getY(DistanceUnit.MM));
        }else{
            updateCurrentPointWithoutCamera();
            setPointFromChassis(robotPoint);
        }
    }

    public void setCurrentTurretHeading(){
        currentTurretHeading = turretPosition.getHeading(AngleUnit.DEGREES);
    }
    public Point getTurretPoint(){
        return turretPoint;
    }

    public void setPointFromChassis(Point robotPoint){
        double xOffset = Math.cos(TURRET_ROBOT_OFFSET);
        double yOffset = Math.sin(TURRET_ROBOT_OFFSET);
        turretPoint = new Point(robotPoint.x+xOffset,robotPoint.y+yOffset);
    }

    public void update(){
        updatePosition();
        setCurrentTurretHeading();
        setTargetTurretHeading();
        pidOut = updatePID();
        setServoPosition();
    }
}