package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double P;
    double I;
    double D;
    public PID(double P, double I, double D){
        this.P=P;
        this.I=I;
        this.D=D;
    }

    double integral = 0;
    double lastError = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    long lastTime;

    public double update(double error){
        // 时间
        long now = elapsedTime.nanoseconds();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        // PID
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double value =
                P * error + I * integral + D * derivative;
        lastError = error;
        return value;
    }

    public void limitIntegral(double limit){

    }


}
