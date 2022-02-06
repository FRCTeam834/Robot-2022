// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("camera");
    private double yaw, pitch, skew, distance = yaw = pitch = skew = 0.0;
    private boolean targetExists = false;

    private double vph;
    private double vpw;
    private Rotation2d horizontalPlaneToLens;
    private double lensHeightMeters;

    public Vision() {}

    public double getYaw() { return yaw; }
    public double getPitch() { return pitch; }
    public double getSkew() { return skew; }
    public double getDistance() { return distance; }
    public boolean hasTarget() { return targetExists; }

    // return list of list for parseTargetCorners
    private List<List<TargetCorner>> getTargetCorners() {
        // I think this is cached
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        if(!pipelineResult.hasTargets()) return null;

        List<List<TargetCorner>> ret = new ArrayList<>();
        List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
        
        for(PhotonTrackedTarget target : targets) {
            ret.add(target.getCorners());
        }

        return ret;
    }

    private List<TargetCorner> parseTargetCorners() {
        List<List<TargetCorner>> cornerData = getTargetCorners();
        if(cornerData == null) return null;

        List<TargetCorner> ret = new ArrayList<>();
        
        // assumes target contours are rectangles
        for(List<TargetCorner> corners : cornerData) {
            TargetCorner p1 = corners.get(0);
            TargetCorner p2 = corners.get(1);
            TargetCorner p3 = corners.get(2);
            TargetCorner p4 = corners.get(3);
            
            // following canvas axises (down is larger)
            double midY = (p1.y + p2.y + p3.y + p4.y) / 4;

            if(p1.y < midY) ret.add(p1);
            if(p2.y < midY) ret.add(p2);
            if(p3.y < midY) ret.add(p3);
            if(p4.y < midY) ret.add(p4);
        }
        
        return ret;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        
        if (!pipelineResult.hasTargets()) {
            targetExists = false;
            return;
        }

        PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
        yaw = bestTarget.getYaw();
        pitch = bestTarget.getPitch();
        skew = bestTarget.getSkew();

        distance = PhotonUtils.calculateDistanceToTargetMeters(
            Parameters.shooter.camera.HEIGHT,
            Parameters.shooter.camera.TARGET_HEIGHT,
            Units.degreesToRadians(Parameters.shooter.camera.PITCH),
            Units.degreesToRadians(bestTarget.getPitch())
        );
        targetExists = true;
    }
}
