// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class HomeIntake extends CommandBase {
    Timer timer = new Timer();
    // Creates a new HomeIntake.
    public HomeIntake() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.intakeWinch);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Disable the soft limits
        RobotContainer.intakeWinch.disableSoftLimits();

        // Set the homing current
        RobotContainer.intakeWinch.setCurrentLimit(Parameters.intake.spool.HOME_CURRENT);
        RobotContainer.intakeWinch.set(-Parameters.intake.spool.HOME_SPEED);

        // Start the timeout timer
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the timer
        timer.reset();
        timer.stop();

        // Stop the motor
        // ! THIS NEEDS TO BE DONE TO PREVENT OVERDRIVING THE MOTOR
        RobotContainer.intakeWinch.stop();

        // Sets it back to standard current limit
        RobotContainer.intakeWinch.setCurrentLimit(Parameters.intake.spool.MOTOR_CURRENT_LIMIT);

        // If the command wasn't interrupted (like another command needing the hood), then we can
        // say that we're at the home position of the shooter
        if (!interrupted) {
            RobotContainer.intakeWinch.setCurrentDistance(Parameters.intake.spool.HOME_DISTANCE);
            RobotContainer.intakeWinch.setDesiredDistance(Parameters.intake.spool.DOWN_DISTANCE);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (RobotContainer.intakeWinch.getMotorCurrent()
                        > Parameters.intake.spool.HOMING_CURRENT_THRESHOLD)
                || timer.hasElapsed(5);
    }
}
