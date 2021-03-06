package frc.robot.utilityClasses.interpolation;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class ShotParams {

    // Creating variables
    double angle;
    double speed;

    // Overloading the constructor
    public ShotParams(double speed, double angle) {
        this.angle = angle;
        this.speed = speed;
    }

    public boolean equals(ShotParams other, double angTol, double spdTol) {
        if (Math.abs(angle - other.angle) <= angTol && Math.abs(speed - other.speed) <= spdTol) {
            return true;
        } else {
            return false;
        }
    }

    // Linear interpolation code, taken from:
    // https://medium.com/swlh/youre-using-lerp-wrong-73579052a3c3
    public double linearInterpolation(double y1, double y2, double t) {
        return y1 + ((y2 - y1) * t);
    }

    public ShotParams interpolate(ShotParams end, double t) {
        return new ShotParams(
                linearInterpolation(this.speed, end.speed, t),
                (linearInterpolation(this.angle, end.angle, t)));
    }

    // Returns the shot angle (deg)
    public double getAngle() {
        return angle;
    }

    // Returns the speed (m/s)
    public double getSpeed() {
        return speed;
    }

    public String toString() {
        return String.format("S: %f | A: %f", speed, angle);
    }
}
