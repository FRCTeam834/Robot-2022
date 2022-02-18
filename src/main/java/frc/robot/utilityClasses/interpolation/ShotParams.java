package frc.robot.utilityClasses.interpolation;


// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class ShotParams {

    // Creating variables
    double angle;
    double speed;

    // Overloading the constructor
    public ShotParams(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }


    public boolean equals(ShotParams other, double tolerance) {
        if (Math.abs(angle - other.angle) <= tolerance
                && Math.abs(speed - other.speed) <= tolerance) {
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
                linearInterpolation(this.angle, end.angle, t),
                (linearInterpolation(this.speed, end.speed, t)));
    }

}
