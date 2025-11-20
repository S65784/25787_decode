package org.firstinspires.ftc.teamcode.pedroPathing.vision.QuickScope;

public class ArcherLogic {

    // --- 所有常量在此处硬编码 ---
    // 物理常量
    private static final double GRAVITY_MS2 = 9.81;
    private static final double AIR_DENSITY = 1.225;
    private static final double HEIGHT_M = 1.065;
    private static final double MASS_KG = 0.012;
    private static final double DRAG_COEFFICIENT = 0.25;
    private static final double CROSS_SECTIONAL_AREA_M2 = 0.00928;
    // 计算参数
    private static final double MIN_ANGLE_DEG = 55.0;
    private static final double MAX_ANGLE_DEG = 90.0;
    private static final double ANGLE_SEARCH_STEP = 1.0;
    private static final double VELOCITY_SEARCH_STEP = 0.1;
    private static final int MAX_VELOCITY_TRIES = 500;
    private static final int BISECTION_ITERATIONS = 8;
    private static final double HIT_TOLERANCE_M = 0.055;
    private static final double TIME_STEP_S = 0.006;
    // 硬件常量
    private static final double MOTOR_RPM_LOSS_FACTOR_PERCENT = -5; //TODO: 根据测试结果调整增益
    private static final double FRICTION_WHEEL_DIAMETER_M = 0.072;
    // 场地常量
    private static final double REAL_FIELD_SIZE_INCHES = 141.170031;
    private static final double INCHES_TO_METERS = 0.0254;
    // 硬编码的目标坐标
    private static final Vector2D TAG_BLUE = new Vector2D(0.080, 0.942);
    private static final Vector2D TAG_RED = new Vector2D(0.920, 0.942);

    /**
     * 这是唯一需要从外部调用的公共方法。
     * @param params 包含机器人所有当前状态的参数对象
     * @return 计算出的发射方案，如果无解则返回null
     */
    public LaunchSolution calculateSolution(CalculationParams params) {
        Vector2D targetPos = params.targetAlliance.equals("Red") ? TAG_RED : TAG_BLUE;
        Vector2D robotPos = new Vector2D(params.robotX_norm, params.robotY_norm);

        Vector2D vectorToTarget = targetPos.subtract(robotPos);
        double distNorm = vectorToTarget.magnitude();
        double distanceM = distNorm * REAL_FIELD_SIZE_INCHES * INCHES_TO_METERS;
        double targetDirRad = Math.atan2(vectorToTarget.y, vectorToTarget.x);

        double vVehicleMs = params.vehicleSpeedMs;
        double vehicleDirRad = Math.toRadians(params.vehicleDirectionDeg);
        Vector2D vVehicleVec = new Vector2D(vVehicleMs * Math.cos(vehicleDirRad), vVehicleMs * Math.sin(vehicleDirRad));

        LaunchSolution minVSol = null;
        double lastMinLauncherV = Double.POSITIVE_INFINITY;

        for (double pva = MIN_ANGLE_DEG; pva <= MAX_ANGLE_DEG; pva += ANGLE_SEARCH_STEP) {
            Double predV = estimateInitialVelocity(pva, distanceM, HEIGHT_M);
            if (predV == null) continue;

            double lowV = predV, highV = 0.0;
            boolean found = false;
            for (int i = 0; i < MAX_VELOCITY_TRIES; i++) {
                double testV = lowV + i * VELOCITY_SEARCH_STEP;
                if (runSimulation(pva, testV, distanceM) > HEIGHT_M) {
                    highV = testV;
                    lowV = Math.max(predV, testV - VELOCITY_SEARCH_STEP);
                    found = true;
                    break;
                }
            }
            if (!found) continue;

            for (int i = 0; i < BISECTION_ITERATIONS; i++) {
                double midV = (lowV + highV) / 2.0;
                if (runSimulation(pva, midV, distanceM) > HEIGHT_M) highV = midV; else lowV = midV;
            }

            double projectileTotalVelocity = highV;
            double finalH = runSimulation(pva, projectileTotalVelocity, distanceM);

            if (Math.abs(finalH - HEIGHT_M) <= HIT_TOLERANCE_M) {
                double vProjectileHMag = projectileTotalVelocity * Math.cos(Math.toRadians(pva));
                double vProjectileV = projectileTotalVelocity * Math.sin(Math.toRadians(pva));
                Vector2D vProjectileHVec = new Vector2D(vProjectileHMag * Math.cos(targetDirRad), vProjectileHMag * Math.sin(targetDirRad));
                Vector2D vLauncherHVec = vProjectileHVec.subtract(vVehicleVec);
                double vLauncherHMag = vLauncherHVec.magnitude();
                double aimAzimuthRad = Math.atan2(vLauncherHVec.y, vLauncherHVec.x) - Math.PI / 2;
                double launcherVelocity = Math.sqrt(vLauncherHMag * vLauncherHMag + vProjectileV * vProjectileV);
                double launcherAngleDeg = Math.toDegrees(Math.atan2(vProjectileV, vLauncherHMag));

                if (launcherVelocity < lastMinLauncherV) {
                    lastMinLauncherV = launcherVelocity;
                    double motorRpm = calculateMotorRpm(launcherVelocity);
                    minVSol = new LaunchSolution(launcherVelocity, launcherAngleDeg, Math.toDegrees(aimAzimuthRad), motorRpm);
                } else {
                    break;
                }
            }
        }
        return minVSol;
    }

    private double calculateMotorRpm(double velocityMs) {
        if (velocityMs <= 0) return 0;
        double theoreticalRpm = (velocityMs * 60) / (Math.PI * FRICTION_WHEEL_DIAMETER_M);
        return theoreticalRpm * (1 + MOTOR_RPM_LOSS_FACTOR_PERCENT / 100.0);
    }

    private Double estimateInitialVelocity(double angleDeg, double targetX, double targetY) {
        double angleRad = Math.toRadians(angleDeg);
        double cosA = Math.cos(angleRad);
        double denominator = 2 * cosA * cosA * (targetX * Math.tan(angleRad) - targetY);
        return (denominator > 0) ? Math.sqrt((GRAVITY_MS2 * targetX * targetX) / denominator) : null;
    }

    private double runSimulation(double launchAngleDeg, double initialVelocityMs, double distanceM) {
        double angleRad = Math.toRadians(launchAngleDeg);
        double vx = initialVelocityMs * Math.cos(angleRad), vy = initialVelocityMs * Math.sin(angleRad);
        double x = 0.0, y = 0.0;
        double dragFactor = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * CROSS_SECTIONAL_AREA_M2;
        double gravityForceY = -MASS_KG * GRAVITY_MS2;
        while (true) {
            if ((vx <= 0 && x < distanceM) || (y < 0 && vy < 0)) return -1.0;
            double prevX = x, prevY = y;
            double vSq = vx * vx + vy * vy;
            if (vSq == 0) return -1.0;
            double v = Math.sqrt(vSq);
            double drag = dragFactor * vSq;
            double ax = -drag * (vx / v) / MASS_KG;
            double ay = (gravityForceY - drag * (vy / v)) / MASS_KG;
            vx += ax * TIME_STEP_S; vy += ay * TIME_STEP_S;
            x += vx * TIME_STEP_S; y += vy * TIME_STEP_S;
            if (x >= distanceM) {
                if ((x - prevX) == 0) return y;
                return prevY + (y - prevY) * ((distanceM - prevX) / (x - prevX));
            }
        }
    }
}