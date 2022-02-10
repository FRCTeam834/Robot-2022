package frc.robot.utilityClasses;

import frc.robot.Parameters;

public class GlobalPoint {
    // Everything relative to camera right now
    double z = Parameters.shooter.camera.TARGET_HEIGHT - Parameters.shooter.camera.HEIGHT;

    // Robot relative axises (change later probably)
    double x = Double.NaN;
    double y = Double.NaN;

    public GlobalPoint(double yaw, double pitch) {
        y = z / Math.tan(pitch);
        x = Math.sin(yaw) * y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
