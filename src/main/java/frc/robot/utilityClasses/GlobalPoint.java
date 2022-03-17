package frc.robot.utilityClasses;

import frc.robot.Parameters;

public class GlobalPoint {
    private static double z = Parameters.shooter.camera.TARGET_HEIGHT - Parameters.shooter.camera.CAMERA_HEIGHT;

    private double x;
    private double y;

    public GlobalPoint(double yaw, double pitch) {
        double d = (z / (Math.tan(pitch))) * Math.cos(yaw);

        x = Math.cos(yaw) * d;
        y = Math.sin(yaw) * d;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
