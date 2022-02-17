// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CircleFitter;
import frc.robot.utilityClasses.GlobalPoint;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {

    public PhotonCamera camera;
    private CircleFitter circlefitter;
    private static double yaw, pitch, skew, distance = yaw = pitch = skew = 0.0;
    private boolean targetExists = false;
    private double horizontalFov;
    private double verticalFov;
    private double vph = 2 * Math.tan(horizontalFov / 2);
    private double vpw = 2 * Math.tan(verticalFov / 2);
    private double resolution;
    // Adding *0.5 to resx and resy so resolution isn't accidentally messed up
    // Resolution = Video Width Resolution / 160 (or vwr / 120)
    private double resolutionX = resolution * 160 * 0.5;
    private double resolutionY = resolution * 120 * 0.5;
    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;
    private boolean LEDsOn = false;
    private List<GlobalPoint> globalPoints;

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

    // return list of list for parseTargetCorners
    private List<List<TargetCorner>> getTargetCorners() {
        PhotonPipelineResult pipelineResult = camera.getLatestResult();

        List<List<TargetCorner>> ret = new ArrayList<>();
        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();

        for (PhotonTrackedTarget target : targets) {
            ret.add(target.getCorners());
        }

        return ret;
    }

    private List<TargetCorner> parsedTargetCorners() {
        List<List<TargetCorner>> cornerData = getTargetCorners();

        List<TargetCorner> ret = new ArrayList<>();

        // assumes target contours are rectangles
        for (List<TargetCorner> corners : cornerData) {
            TargetCorner p1 = corners.get(0);
            TargetCorner p2 = corners.get(1);
            TargetCorner p3 = corners.get(2);
            TargetCorner p4 = corners.get(3);

            // following canvas axises (down is larger)
            double midY = (p1.y + p2.y + p3.y + p4.y) / 4;

            if (p1.y < midY) ret.add(p1);
            if (p2.y < midY) ret.add(p2);
            if (p3.y < midY) ret.add(p3);
            if (p4.y < midY) ret.add(p4);
        }

        return ret;
    }

    private List<GlobalPoint> calculateGlobalPoints() {
        List<TargetCorner> targetCorners = parsedTargetCorners();
        List<GlobalPoint> ret = new ArrayList<>();
        // No points detected
        if (targetCorners.size() == 0) return ret;

        for (TargetCorner corner : targetCorners) {
            // + 0.5 for 1 unit pixel plane
            double nx = (1 / resolutionX) * (corner.x - resolutionX + 0.5);
            double ny = (1 / resolutionY) * (resolutionY - corner.y + 0.5);
            // coordinates on imaginary view plane
            double x = vpw / 2 * nx;
            double y = vph / 2 * ny;

            pitch = Math.atan2(1, x);
            yaw = Math.atan2(1, y);
            ret.add(new GlobalPoint(yaw, pitch));
        }

        globalPoints = ret;
        return ret;
    }

    public double[] getTargetCenter() {
        // doesn't really make sense to do anything if there are no points, so return previous
        // calculated center?
        if (calculateGlobalPoints().size() == 0) return CircleFitter.getCircleData();
        CircleFitter.setPoints(calculateGlobalPoints());

        // In Pose implementation just use target x/y as minuend
        return CircleFitter.calculateCircle();
    }

    public List<GlobalPoint> getGlobalPoints() {
        return globalPoints;
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
