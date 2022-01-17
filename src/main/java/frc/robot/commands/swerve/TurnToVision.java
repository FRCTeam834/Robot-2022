// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

// Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.Robot;

import org.photonvision.targeting.PhotonPipelineResult;

public class TurnToVision extends CommandBase {

    public TurnToVision() {
        addRequirements(Robot.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get a list of possible targets
        PhotonPipelineResult targetList = Robot.goalCamera.getLatestResult();

        // Make sure there are targets available, otherwise there could be a null pointer exception
        if (targetList.hasTargets()) {

            // Calculate the new pose needed based off the current one, then move there
            Transform2d transformNeeded = targetList.getBestTarget().getCameraToTarget();
            Pose2d newPose = Robot.driveTrain.getPose2D().transformBy(transformNeeded);
            Robot.driveTrain.trajectoryFollow(
                    newPose, Parameters.driver.currentProfile.maxModVelocity / 2);
        } else {

            // No target, so stop the modules
            // ! THIS IS EXTREMELY IMPORTANT TO PREVENT THE ROBOT FROM BECOMING A BEYBLADE
            Robot.driveTrain.stopModules();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop all of the modules (basically zero their velocities)
        Robot.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Return if the robot has finished the movement yet
        return Robot.driveTrain.finishedMovement();
    }
}
