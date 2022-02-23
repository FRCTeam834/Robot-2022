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

import org.photonvision.targeting.PhotonTrackedTarget;

public class TurnToGoal extends CommandBase {

    // If the command should ever finish
    boolean shouldFinish;

    public TurnToGoal(boolean shouldFinish) {

        // Request the subsystem
        addRequirements(RobotContainer.driveTrain, RobotContainer.vision);

        // Save if we need to finish
        this.shouldFinish = shouldFinish;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Calculate the speeds of the translational joystick
        double rightXSpeed = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX()) * Parameters.driver.maxModVelocity;
        double rightYSpeed = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY()) * Parameters.driver.maxModVelocity;

        // Get a list of possible targets
        PhotonTrackedTarget bestTarget = RobotContainer.vision.getBestTarget();

        // Make sure that we have targets to track
        if (bestTarget != null) {

            // Calculate the raw rotational PID output
            double pidOutput =
                    -RobotContainer.driveTrain.rotationPID.calculate(
                            (bestTarget.getYaw()), 0);

            // Calculate the rotational speed to run at
            double rotationalSpeed =
                    MathUtil.clamp(
                            pidOutput,
                            Parameters.vision.MAX_TURNING_SPEED,
                            -Parameters.vision.MAX_TURNING_SPEED);

            // Drive at the specified speed
            RobotContainer.driveTrain.drive(
                    rightYSpeed, rightXSpeed, Units.degreesToRadians(rotationalSpeed), RobotContainer.fieldCentric);

        } else {
            // starts spinning to search for a target
            // TODO: Fix inefficiencies, use gyro angle to get optimal rotation
            RobotContainer.driveTrain.drive(
                    rightYSpeed, rightXSpeed, Units.degreesToRadians(Parameters.vision.SPIN_SPEED), RobotContainer.fieldCentric);
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

        // Check if the command should ever finish
        if (shouldFinish) {

            // Return if the robot has finished the movement yet
            return RobotContainer.vision.isLinedUp();
        } else {
            return false;
        }
    }
}
