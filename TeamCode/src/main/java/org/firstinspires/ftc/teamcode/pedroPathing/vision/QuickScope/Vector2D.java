package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

public class Vector2D {
    public double x, y;

    public Vector2D(double x, double y) { this.x = x; this.y = y; }

    public Vector2D subtract(Vector2D other) { return new Vector2D(this.x - other.x, this.y - other.y); }

    public double magnitude() { return Math.sqrt(x * x + y * y); }
}