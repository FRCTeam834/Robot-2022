// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    public static PhotonCamera camera;
    private static double yaw, pitch, skew, distance = yaw = pitch = skew = 0.0;
    private boolean targetExists = false;
    private double vph;
    private double vpw;
    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;
    private static boolean LEDsOn;

    public Vision() {

        // Set up the PhotonVision camera
        camera = new PhotonCamera(Parameters.vision.CAMERA_NAME);

        // Put the camera in driver mode (so that it's not processing without the LEDs)
        camera.setDriverMode(true);

        // Turn the LEDs off (they are super bright and annoying)
        camera.setLED(VisionLEDMode.kOff);

        // Set that the LEDs are off
        LEDsOn = false;
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

        // Get the best target
        PhotonTrackedTarget bestTarget = getBestTarget();

        // Make sure that the target isn't null (that we have a target)
        if (bestTarget != null) {

            // Check to see if the yaw deviation is below the tolerance
            return (Parameters.vision.YAW_TOLERANCE > Math.abs(bestTarget.getYaw()));
        } else {
            // We don't have a target, so we're not lined up
            return false;
        }
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

    public double getYaw() {
        if (camera.getLatestResult().hasTargets()) return getBestTarget().getYaw();
        else return 0;
    }

    @Override
    public void periodic() {
        //camera.setVersionCheckEnabled(false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            //builder.addDoubleProperty("Yaw", () -> getYaw(), null);
            //builder.addDoubleProperty("Distance", () -> getDistanceToGoal(getBestTarget()), null);
            builder.addBooleanProperty(
                    "hasTargets", () -> camera.getLatestResult().hasTargets(), null);
            builder.addBooleanProperty("isLinedUp", this::isLinedUp, null);
            builder.addDoubleProperty("distance", this::getDistanceToGoal, null);
        }
    }
}
