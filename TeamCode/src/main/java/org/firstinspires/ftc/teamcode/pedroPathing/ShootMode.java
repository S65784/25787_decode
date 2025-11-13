package org.firstinspires.ftc.teamcode.pedroPathing;

public class ShootMode {
    int targetRPM;
    int error;
    public ShootMode(int targetRPM, int error){
    }
    public void shoot(boolean yState){
        Algorithm.shoot(targetRPM,error,true,yState);{
        }
    }
}
