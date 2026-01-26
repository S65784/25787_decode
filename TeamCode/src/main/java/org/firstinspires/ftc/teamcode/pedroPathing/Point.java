package org.firstinspires.ftc.teamcode.pedroPathing;

public class Point {
    double x;
    double y;
    public Point(double x,double y){
        this.x=x;
        this.y=y;
    }

    public static double getAngleFromPoints(Point p1,Point p2){
        double x = p2.x-p1.x;
        double y = p2.y-p1.y;
        return Math.toDegrees(Math.atan2(y,x));
    }
}
