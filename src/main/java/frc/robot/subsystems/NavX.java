/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/8/20
 */
package frc.robot.subsystems;

// Imports
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class NavX extends SubsystemBase {
    /** Creates a new NavX. */
    AHRS navX = new AHRS(SPI.Port.kMXP);

    private float pitchOffset = 0;

    public NavX() {
        navX.calibrate();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Grabs the yaw
    public float getYaw() {
        return -navX.getYaw();
    }

    // Fused heading is like yaw, but on crack
    public float getFusedHeading() {
        return navX.getFusedHeading();
    }

    // The Rotation2D is the big brother of fused heading
    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }

    // Grabs the roll
    public float getRoll() {
        return navX.getRoll();
    }

    // Grabs the pitch
    public float getPitch() {
        return navX.getPitch() + pitchOffset;
    }

    // Gets the x displacement
    public float getDisplacementX() {
        return navX.getDisplacementX();
    }

    // Gets the y displacement
    public float getDisplacementY() {
        return navX.getDisplacementY();
    }

    // Gets the z displacement
    public float getDisplacementZ() {
        return navX.getDisplacementZ();
    }

    // Gets the current degrees
    public float getCompassHeading() {
        return navX.getCompassHeading();
    }

    // Resets the NavX's yaw axis to zero
    public void resetYaw() {
        navX.zeroYaw();
    }

    // Resets the NavX's yaw axis to zero
    public void resetPitch() {
        pitchOffset = -getPitch();
    }

    public void resetDisplacement() {
        resetDisplacement();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("NavX");
            builder.addDoubleProperty("Angle", navX::getYaw, null);
        }
    }
}
