// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notSwerve;



import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class ExtendClimber extends CommandBase {
    /** Creates a new ExtendClimber. */
    public ExtendClimber() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (RobotContainer.climber.getFrontPosition()
                < RobotContainer.climber.getBackPosition() - Parameters.climber.ALLOWABLE_DEVIATION) {

            RobotContainer.climber.setFrontMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getFrontPosition(),
                            RobotContainer.climber.getBackPosition()));

            RobotContainer.climber.setBackMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getBackPosition(), Parameters.climber.SETPOINT));

        } else if (RobotContainer.climber.getFrontPosition()
                > RobotContainer.climber.getBackPosition() - Parameters.climber.ALLOWABLE_DEVIATION) {

            RobotContainer.climber.setBackMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getBackPosition(),
                            RobotContainer.climber.getFrontPosition()));

            RobotContainer.climber.setFrontMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getFrontPosition(),
                            Parameters.climber.SETPOINT));

        } else {

            RobotContainer.climber.setFrontMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getFrontPosition(),
                            Parameters.climber.SETPOINT));

            RobotContainer.climber.setBackMotor(
                    RobotContainer.climber.getPIDValue(
                            RobotContainer.climber.getBackPosition(), Parameters.climber.SETPOINT));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.climber.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (RobotContainer.climber.getFrontPosition() >= Parameters.climber.SETPOINT
                && RobotContainer.climber.getBackPosition() >= Parameters.climber.SETPOINT);
    }
}
