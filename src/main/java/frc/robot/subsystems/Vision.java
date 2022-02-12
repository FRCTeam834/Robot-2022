// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    public PhotonCamera camera;
    private static double yaw, pitch, skew, distance = yaw = pitch = skew = 0.0;
    private boolean targetExists = false;
    private double vph;
    private double vpw;
    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;
    private boolean LEDsOn = false;

    public Vision() {

        // Set up the PhotonVision camera
        camera = new PhotonCamera(Parameters.vision.CAMERA_NAME);

        // Turn the LEDs off (they are super bright and annoying)
        camera.setLED(VisionLEDMode.kOff);
    }

    public void turnLEDsOn() {
        if (!LEDsOn) {
            camera.setLED(VisionLEDMode.kOn);
        }
    }

    public void turnLEDsOff() {
        if (LEDsOn) {
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
        if (!LEDsOn) {
            turnLEDsOn();
        }

        // Return the best target from the camera
        return camera.getLatestResult().getBestTarget();
    }

    /**
     * Checks if the robot is lined up with the best target
     *
     * @return Is the robot lined up?
     */
    public boolean isLinedUp() {

        // Check to see if the yaw deviation is below the tolerance
        return (Parameters.vision.YAW_TOLERANCE > getBestTarget().getYaw());
    }

    /*
    public something getPositionFromVision() {
        // returnval = circleFitAlgorithm(getGlobalPoints());
        // xpos = Parameters.shooter.camera.TARGET_X - returnval[0];
        // ypos = Parameters.shooter.camera.TARGET_Y - returnval[1];
    }
    */

    private List<GlobalPoint> getGlobalPoints() {
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        if (!pipelineResult.hasTargets()) return null;

        List<GlobalPoint> ret = new ArrayList<>();
        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();

        for (PhotonTrackedTarget target : targets) {
            GlobalPoint gp =
                    new GlobalPoint(
                            Units.degreesToRadians(target.getYaw()),
                            Units.degreesToRadians(
                                    Parameters.shooter.camera.PITCH + target.getPitch()));
            ret.add(gp);
        }

        return ret;
    }

    public static double getDistanceToGoal(PhotonTrackedTarget bestTarget) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                Parameters.vision.CAMERA_HEIGHT,
                Parameters.vision.GOAL_HEIGHT,
                Units.degreesToRadians(Parameters.vision.CAMERA_PITCH),
                Units.degreesToRadians(bestTarget.getPitch()));
    }

    @Override
    public void periodic() {}
}
