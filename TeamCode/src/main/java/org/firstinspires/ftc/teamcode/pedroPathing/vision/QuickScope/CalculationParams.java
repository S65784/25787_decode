package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

public class CalculationParams {
    public final double robotX_norm;
    public final double robotY_norm;
    public final double vehicleSpeedMs;
    public final double vehicleDirectionDeg;
    public final String targetAlliance; // "Red" or "Blue"

    public CalculationParams(double robotX_norm, double robotY_norm, double vehicleSpeedMs, double vehicleDirectionDeg, String targetAlliance) {
        this.robotX_norm = robotX_norm;
        this.robotY_norm = robotY_norm;
        this.vehicleSpeedMs = vehicleSpeedMs;
        this.vehicleDirectionDeg = vehicleDirectionDeg;
        this.targetAlliance = targetAlliance;
    }
}