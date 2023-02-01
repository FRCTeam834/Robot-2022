// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

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
    public void execute() {}

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
