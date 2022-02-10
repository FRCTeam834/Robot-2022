package frc.robot.utilityClasses;

import frc.robot.Parameters;

public class GlobalPoint {
    // Everything relative to camera right now
    double z = Parameters.shooter.camera.TARGET_HEIGHT - Parameters.shooter.camera.HEIGHT;

    double x = Double.NaN;
    double y = Double.NaN;

    public GlobalPoint(double yaw, double pitch) {
        double d = z / Math.tan(pitch);
        // If yaw is from atan2, then this is redundant and use x = cos and y = sin
        // Quadrant I and III
        if(
            yaw >= 0 && yaw < Math.PI / 2 ||
            yaw >= Math.PI && yaw < 3 * Math.PI / 2
        ) {
            x = Math.sin(yaw) * d;
            y = Math.cos(yaw) * d;
        } else {
            x = Math.cos(yaw) * d;
            y = Math.sin(yaw) * d;
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
