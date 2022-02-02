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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("camera");
    private double yaw, pitch, skew, distance = yaw = pitch = skew = 0.0;
    private boolean targetExists = false;
    private double vph;
    private double vpw;
    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;

    public Vision() {}

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getSkew() {
        return skew;
    }

    public double getDistance() {
        return distance;
    }

    public boolean hasTarget() {
        return targetExists;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult targetList = camera.getLatestResult();

        if (targetList.hasTargets()) {
            PhotonTrackedTarget target = targetList.getBestTarget();
            targetExists = true;
            yaw = target.getYaw();
            pitch = target.getPitch();
            skew = target.getSkew();
            distance =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            Parameters.shooter.camera.HEIGHT,
                            Parameters.shooter.camera.TARGET_HEIGHT,
                            Units.degreesToRadians(Parameters.shooter.camera.PITCH),
                            Units.degreesToRadians(target.getPitch()));
        } else {
            targetExists = false;
        }
    }
}
