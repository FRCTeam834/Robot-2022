// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

import org.photonvision.targeting.PhotonPipelineResult;

public class TurnToVision extends CommandBase {

    public TurnToVision() {
        addRequirements(RobotContainer.driveTrain, RobotContainer.vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get a list of possible targets
        PhotonPipelineResult targetList = RobotContainer.vision.camera.getLatestResult();

        // Make sure that we have targets to track
        if (targetList.hasTargets()) {

            // Calculate the raw rotational PID output
            double pidOutput =
                    -RobotContainer.driveTrain.rotationPID.calculate(
                            (targetList.getBestTarget().getYaw()), 0);

            // Calculate the rotational speed to run at
            double rotationalSpeed =
                    MathUtil.clamp(
                            pidOutput,
                            Parameters.vision.MAX_TURNING_SPEED,
                            -Parameters.vision.MAX_TURNING_SPEED);

            // Drive at the specified speed
            RobotContainer.driveTrain.drive(
                    0.0, 0.0, Units.degreesToRadians(rotationalSpeed), false);

        } else {
            // starts spinning to search for a target
            // TODO: Fix inefficiencies, use gyro angle to get optimal rotation
            RobotContainer.driveTrain.drive(
                    0, 0, Units.degreesToRadians(Parameters.vision.SPIN_SPEED), false);
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
        return RobotContainer.vision.isLinedUp();
    }
}
