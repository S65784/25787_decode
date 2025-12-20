package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShootMode {
    int targetRPM;
    int error;
    double servoPosition;
    double blenderPower;
    double intakePower;
    PIDFCoefficients pid;
//    double lsServoPosition = servoPosition;
//    double rsServoPosition = 1-lsServoPosition;
    public ShootMode(int targetRPM, int error, double servoPosition, double blenderPower, double intakePower, PIDFCoefficients pid){
        this.targetRPM = targetRPM;
        this.error = error;
        this.servoPosition = servoPosition;
        this.blenderPower = blenderPower;
        this.intakePower = intakePower;
        this.pid = pid;
    }

//    ShootMode ShootMode1=
    public void setServos(){
        Algorithm.ls.setPosition(servoPosition);
        Algorithm.rs.setPosition(1-servoPosition);
    }

    public void shoot(boolean yState){
        Algorithm.shoot(targetRPM,error,blenderPower,true,yState);
        setServos();
        setPid();
    }
    public void preShoot(){
        Algorithm.shoot(targetRPM,error,blenderPower,true,false);
        setServos();
        setPid();
    }
    public void shootTime(boolean state, boolean yState,int millitime){
        Algorithm.shootTime(targetRPM,error,blenderPower,state,yState,millitime);
        setServos();
        setPid();
    }

    public void shootTime(int milli){
        shootTime(true,true,milli);
    }
    public void shootCheckOnce(boolean state, boolean yState){
        Algorithm.shootCheckOnce(targetRPM,error,blenderPower,intakePower,state,yState);
        setServos();
        setPid();
    }

    public void shootCheckOnceTime(boolean state, boolean yState,int milli){
        Algorithm.shootCheckOnceTime(targetRPM,error,blenderPower,intakePower,state,yState,milli);
        setServos();
        setPid();
    }

    public void shootCheckOnceTime(int milli) {
        shootCheckOnceTime(true,true,milli);
    }

    public void setPid(){
        Algorithm.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Algorithm.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
    }
}
