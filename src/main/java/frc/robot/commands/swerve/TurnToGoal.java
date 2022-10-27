// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

import org.photonvision.targeting.PhotonTrackedTarget;

public class TurnToGoal extends CommandBase {

    PIDController rotationalPID = new PIDController(3, 0, 0);
    double omega = 0;

    public TurnToGoal() {

        // Set up the rotational PID controller
        rotationalPID.enableContinuousInput(0, 360);
        rotationalPID.setTolerance(.5);

        // Request the drivetrain
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the joystick values (for translational movement)
        // double rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
        // double rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());

        // Get the vision target
        PhotonTrackedTarget latestResult = RobotContainer.vision.getBestTarget();

        // Make sure that we actually have a target
        if (latestResult == null) {

            // We shouldn't be moving if there isn't a target
            omega = 0;
        } else {
            // Use the target to feed the PID controller
            // We need to convert from deg to rad because drive() uses radians
            // Clamping is done to keep the movement within reasonable turning rates
            omega =
                    Math.toRadians(
                            MathUtil.clamp(
                                    rotationalPID.calculate(
                                            latestResult.getYaw(), Parameters.vision.YAW_OFFSET),
                                    -Parameters.vision.MAX_TURNING_SPEED,
                                    Parameters.vision.MAX_TURNING_SPEED));
        }

        RobotContainer.driveTrain.drive(0, 0, omega, true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop all of the modules (basically zero their velocities)
        RobotContainer.driveTrain.zeroVelocities();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rotationalPID.atSetpoint();
    }
}
