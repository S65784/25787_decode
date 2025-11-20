package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

public class LaunchSolution {
    public final double launcherVelocity;
    public final double launcherAngle; // 俯仰角 (Pitch)
    public final double aimAzimuthDeg; // 偏航角 (Yaw)
    public final double motorRpm;

    public LaunchSolution(double launcherVelocity, double launcherAngle, double aimAzimuthDeg, double motorRpm) {
        this.launcherVelocity = launcherVelocity;
        this.launcherAngle = launcherAngle;
        this.aimAzimuthDeg = aimAzimuthDeg;
        this.motorRpm = motorRpm;
    }
}