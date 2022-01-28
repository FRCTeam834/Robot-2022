// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class ExtendClimberOneMotor extends CommandBase {
    /** Creates a new ExtendClimberOneMotor. */
    public ExtendClimberOneMotor() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.climber.setFrontMotor(
                RobotContainer.climber.getPIDValue(
                        RobotContainer.climber.getFrontPosition(), Parameters.climber.SETPOINT));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.climber.setFrontMotor(
                RobotContainer.climber.getPIDValue(
                        RobotContainer.climber.getFrontPosition(), Parameters.climber.SETPOINT));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (RobotContainer.climber.getFrontPosition() == Parameters.climber.SETPOINT) {
            return false;
        }
    }
}
