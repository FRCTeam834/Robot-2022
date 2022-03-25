// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class HomeHood extends CommandBase {
    /** Creates a new SetBase. */
    public HomeHood() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Disable the soft limits, we don't know where we are
        RobotContainer.hood.disableSoftLimits();

        // Set the current way way way down in case the limit switch doesn't work
        RobotContainer.hood.setMotorCurrent(Parameters.hood.HOME_CURRENT);
        RobotContainer.hood.set(Parameters.hood.HOME_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the motor
        // ! THIS NEEDS TO BE DONE TO PREVENT OVERDRIVING THE MOTOR
        RobotContainer.hood.stop();

        // Set the current back to the regular level
        RobotContainer.hood.setMotorCurrent(Parameters.hood.CURRENT_LIMIT);

        // If the command wasn't interrupted (like another command needing the hood), then we can
        // say that we're at the home position of the shooter
        if (!interrupted) {
            RobotContainer.hood.setCurrentAngle(Parameters.hood.HOME_ANGLE);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.hood.getLSValue();
    }
}
