package org.firstinspires.ftc.teamcode.pedroPathing;

public class ShootMode {
    int targetRPM;
    int error;
    double servoPosition;
//    double lsServoPosition = servoPosition;
//    double rsServoPosition = 1-lsServoPosition;
    public ShootMode(int targetRPM, int error, double servoPosition){
        this.targetRPM = targetRPM;
        this.error = error;
        this.servoPosition = servoPosition;
    }

//    ShootMode ShootMode1=
    public void setServos(){
        Algorithm.ls.setPosition(servoPosition);
        Algorithm.rs.setPosition(1-servoPosition);
    }

    public void shoot(boolean yState){
        Algorithm.shoot(targetRPM,error,true,yState);
        setServos();
    }
    public void preShoot(){
        Algorithm.shoot(targetRPM,error,true,false);
        setServos();
    }
    public void shootTime(boolean state, boolean yState,int millitime){
        Algorithm.shootTime(targetRPM,error,state,yState,millitime);
        setServos();
    }

    public void shootTime(int milli){
        shootTime(true,true,milli);
    }
    public void shootCheckOnce(boolean state, boolean yState){
        Algorithm.shootCheckOnce(targetRPM,error,state,yState);
        setServos();
    }

    public void shootCheckOnceTime(boolean state, boolean yState,int milli){
        Algorithm.shootCheckOnceTime(targetRPM,error,state,yState,milli);
        setServos();
    }

    public void shootCheckOnceTime(int milli) {
        shootCheckOnceTime(true,true,milli);
    }

}
