// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilityClasses.interpolation;

/** Add your docs here. */
public class ShotParams {

    // Creating variables
    double angle;
    double speed;

    // Creating the constructor
    public ShotParams() {
        angle = 0;
        speed = 0;
    }

    // Overloading the constructor
    public ShotParams(double angle) {
        angle = this.angle;
        speed = 0;
    }

    // Overloading the constructor
    public ShotParams(double angle, double speed) {
        angle = this.angle;
        speed = this.speed;
    }

    private double getAngle() {
        return angle;
    }

    private double getSpeed() {
        return speed;
    }

    public boolean equals(ShotParams other, double tolerance) {
        if (Math.abs(angle - other.getAngle()) <= tolerance && Math.abs(speed - other.getSpeed()) <= tolerance) {
            return true;
        }
        else {
            return false;
        }
    }

    // Linear interpolation code, taken from: https://medium.com/swlh/youre-using-lerp-wrong-73579052a3c3
    public double linearInterpolation(double y1, double y2, double t){
        return y1 + t*(y2 -y1);
    }

    public ShotParams interpolate(ShotParams a, double t) {
        return new ShotParams(linearInterpolation(this.angle, a.getAngle(), t), (linearInterpolation(this.speed, a.getSpeed(), t)));
    }

}
