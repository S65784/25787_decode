package org.firstinspires.ftc.teamcode.pedroPathing;

public class ShootMode {
    int targetRPM;
    int error;
    double lsServoPosition;
    double rsServoPosition = 1-lsServoPosition;
    public ShootMode(int targetRPM, int error, double servoPosition){
        this.targetRPM = targetRPM;
        this.error = error;
        lsServoPosition = servoPosition;
    }

//    ShootMode ShootMode1=
    public void shoot(boolean yState){
        Algorithm.shoot(targetRPM,error,true,yState);
        Algorithm.ls.setPosition(lsServoPosition);
        Algorithm.rs.setPosition(rsServoPosition);
    }
    public void shootTime(boolean state, int millitime){
        Algorithm.shootTime(targetRPM,error,state,millitime);
        Algorithm.ls.setPosition(lsServoPosition);
        Algorithm.rs.setPosition(rsServoPosition);
    }
}
