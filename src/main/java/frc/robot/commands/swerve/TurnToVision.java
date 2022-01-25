// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

import org.photonvision.targeting.PhotonPipelineResult;

public class TurnToVision extends CommandBase {

    public TurnToVision() {
        addRequirements(RobotContainer.driveTrain, Superstructure.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get a list of possible targets
        PhotonPipelineResult targetList = Superstructure.vision.camera.getLatestResult();

        if (targetList.hasTargets()) {
            RobotContainer.driveTrain.drive(
                    0.0,
                    0.0,
                    MathUtil.clamp(
                            RobotContainer.driveTrain.rotationPID.calculate(
                                    (targetList.getBestTarget().getYaw()), 0),
                            .75,
                            -.75),
                    false);

        } else {
            // starts spinning to search for a target
            // TODO: Fix inefficiencies, use gyro angle to get optimal rotation
            RobotContainer.driveTrain.drive(0, 0, .25, false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop all of the modules (basically zero their velocities)
        RobotContainer.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Return if the robot has finished the movement yet
        return RobotContainer.driveTrain.rotationPID.atSetpoint();
    }
}
