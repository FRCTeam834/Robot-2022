// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.MovingAverage;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    public static PhotonCamera camera;
    private static boolean LEDsOn;
    public static boolean hasTargets;
    private MovingAverage distAverage;
    private MovingAverage yawAverage;

    public Vision() {

        // Set up the PhotonVision camera
        camera = new PhotonCamera(Parameters.vision.CAMERA_NAME);

        // Put the camera in driver mode (so that it's not processing without the LEDs)
        camera.setDriverMode(true);

        // Turn the LEDs off (they are super bright and annoying)
        camera.setLED(VisionLEDMode.kOff);

        // Set that the LEDs are off
        LEDsOn = false;

        // Set up the moving average filter
        distAverage = new MovingAverage(Parameters.vision.AVG_COUNT);
        yawAverage = new MovingAverage(Parameters.vision.YAW_AVG_COUNT);
    }

    public void turnLEDsOn() {
        if (!LEDsOn) {
            camera.setDriverMode(false);
            camera.setLED(VisionLEDMode.kOn);
        }
    }

    public void turnLEDsOff() {
        if (LEDsOn) {
            camera.setDriverMode(true);
            camera.setLED(VisionLEDMode.kOff);
        }
    }

    /**
     * Makes sure that the LEDs are on, then gets the best target from the vision system
     *
     * @return The best found target
     */
    public PhotonTrackedTarget getBestTarget() {

        // Make sure that the LEDs are on (can't detect colors correctly without them)
        turnLEDsOn();

        // Get the latest result from the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // Return the best target from the camera (if there are targets)
        if (result.hasTargets()) {
            return result.getBestTarget();
        } else {
            return null;
        }
    }

    /**
     * Checks if the robot is lined up with the best target
     *
     * @return Is the robot lined up?
     */
    public boolean isLinedUp() {

        // Get the target yaw
        double targetYaw = getYaw();

        // Make sure that there is a target
        if (targetYaw != Double.NaN) {

            // Check to see if the yaw deviation is below the tolerance
            return (Parameters.vision.YAW_TOLERANCE
                    > Math.abs(targetYaw - Parameters.vision.YAW_OFFSET));
        } else {
            // We don't have a target, so we're not lined up
            return false;
        }
    }

    public void flushDistAvg() {
        distAverage.clearPts();
    }

    public void flushYawAvg() {
        yawAverage.clearPts();
    }

    public double getDistanceToGoal(PhotonTrackedTarget bestTarget) {
        if (bestTarget != null) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    Parameters.vision.CAMERA_HEIGHT,
                    Parameters.vision.GOAL_HEIGHT,
                    Units.degreesToRadians(Parameters.vision.CAMERA_PITCH),
                    Units.degreesToRadians(bestTarget.getPitch()));
        } else {
            return 0;
        }
    }

    public double getDistanceToGoal() {
        return getDistanceToGoal(getBestTarget());
    }

    public double getDistanceToGoalInches() {
        return distAverage.addPt(Units.metersToInches(getDistanceToGoal()));
    }

    private double getShotSpeed() {
        return RobotContainer.interpolatingTable.getShotParam(getDistanceToGoalInches()).getSpeed();
    }

    private double getShotAngle() {
        return RobotContainer.interpolatingTable.getShotParam(getDistanceToGoalInches()).getAngle();
    }

    public double getYaw() {
        // Get the best target
        PhotonTrackedTarget bestTarget = getBestTarget();
        // Only add to the yaw avg if a target is detected, adding 0 will just skew the data
        if (bestTarget != null) return yawAverage.addPt(bestTarget.getYaw());
        else return Double.NaN; // 0 yaw is possible so NaN
    }

    public boolean hasTargets() {
        return hasTargets;
    }

    @Override
    public void periodic() {
        hasTargets = camera.getLatestResult().hasTargets();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            // builder.addDoubleProperty("Yaw", () -> getYaw(), null);
            // builder.addDoubleProperty("Distance", () -> getDistanceToGoal(getBestTarget()),
            // null);
            builder.addBooleanProperty(
                    "hasTargets", () -> camera.getLatestResult().hasTargets(), null);
            builder.addBooleanProperty("isLinedUp", this::isLinedUp, null);
            builder.addDoubleProperty("distance_inches", this::getDistanceToGoalInches, null);
            builder.addDoubleProperty("shot_speed", this::getShotSpeed, null);
            builder.addDoubleProperty("shot_angle", this::getShotAngle, null);
        }
    }
}
