// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
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

    public static PhotonCamera camera;
    private boolean targetExists = false;

    private static double horizontalFov = Parameters.vision.CAMERA_HFOV,
            verticalFov = Parameters.vision.CAMERA_VFOV,
            vph = 2 * Math.tan(horizontalFov / 2),
            vpw = 2 * Math.tan(verticalFov / 2),
            resolutionX = Parameters.vision.CAMERA_RESOLUTION_X,
            resolutionY = Parameters.vision.CAMERA_RESOLUTION_Y,
            hresX = resolutionX / 2,
            hresY = resolutionY / 2;

    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;

    private Pose2d lastPoseFromVision;
    private List<TargetCorner> lastTargetCorners;

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

    /**
     * @return List of corners from vision pipeline
     */
    private List<List<TargetCorner>> getTargetCorners() {
        PhotonPipelineResult pipelineResult = camera.getLatestResult();

        List<List<TargetCorner>> ret = new ArrayList<>();
        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();

        for (PhotonTrackedTarget target : targets) {
            ret.add(target.getCorners());
        }

        return ret;
    }

    /**
     * Parse target corners from vision pipeline so only top corners are kept
     *
     * @return List of parsed target corners, returns null if no targets detected
     */
    private List<TargetCorner> parsedTargetCorners() {
        List<List<TargetCorner>> cornerData = getTargetCorners();
        if (cornerData.size() == 0) return null;

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

        // Camera did not update (`equals` is order senstive but more performant, use if confident
        // Photon retains order)
        // if(ret.equals(lastTargetCorners)) return null;
        if (ret.containsAll(lastTargetCorners)) return null;
        lastTargetCorners = ret;

        return ret;
    }

    /**
     * Get list of global points from target corners
     *
     * @return List of global points, returns null if no vision points exist
     */
    private List<GlobalPoint> calculateGlobalPoints() {
        List<TargetCorner> targetCorners = parsedTargetCorners();
        List<GlobalPoint> ret = new ArrayList<>();
        // No points detected
        if (targetCorners.size() == 0) return null;

        for (TargetCorner corner : targetCorners) {
            double nx = (1 / (hresX)) * (corner.x - hresX);
            double ny = (1 / (hresY)) * (hresY - corner.y);
            // coordinates on imaginary view plane (1 unit away from camera origin)
            double x = vpw / 2.0 * nx;
            double y = vph / 2.0 * ny;

            double yaw = Math.atan(x);
            double pitch = Math.atan(y);

            ret.add(new GlobalPoint(yaw, pitch + Math.toRadians(Parameters.vision.CAMERA_PITCH)));
        }

        return ret;
    }

    /**
     * Get positional data for target center point ! Coords are robot relative and not field
     * relative
     *
     * @return double array with target center data [x, y, radius]
     */
    private double[] headingToTargetCenter() {
        return CircleFitter.calculateCircle(calculateGlobalPoints());
    }

    /**
     * Calculates robot pose using vision data
     *
     * @param facing Rads - angle of camera from 0
     */
    private void calculateRobotPose(Rotation2d facing) {
        double facingInRadians = facing.getRadians();
        double[] heading = headingToTargetCenter();

        if (heading == null) return;

        // Camera relative displacement
        double sx = heading[0];
        double sy = heading[1];

        double globalsx = sx * Math.cos(facingInRadians) - sy * Math.sin(facingInRadians);
        double globalsy = sy * Math.cos(facingInRadians) + sx * Math.sin(facingInRadians);

        double camerax = Parameters.vision.GOAL_X - globalsx;
        double cameray = Parameters.vision.GOAL_Y - globalsy;

        lastPoseFromVision = new Pose2d(camerax, cameray, facing);
    }

    /**
     * Getter for latest pose calculated from vision data
     *
     * @return last pose calculated from vision data
     */
    public Pose2d getLastPoseFromVision() {
        return lastPoseFromVision;
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
        return Units.metersToInches(getDistanceToGoal());
    }

    public double getYaw() {
        // Get the best target
        PhotonTrackedTarget bestTarget = getBestTarget();
        if (bestTarget != null) return bestTarget.getYaw();
        else return 0;
    }

    @Override
    public void periodic() {
        // camera.setVersionCheckEnabled(false);
        calculateRobotPose(RobotContainer.navX.getRotation2d());
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
        }
    }
}
