// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class HomeIntake extends CommandBase {
    /** Creates a new HomeIntake. */
    public HomeIntake() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.intakeSpool.runSpoolMotor(Parameters.intake.spool.HOME_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the motor
        // ! THIS NEEDS TO BE DONE TO PREVENT OVERDRIVING THE MOTOR
        RobotContainer.intakeSpool.stopSpoolMotor();

        // If the command wasn't interrupted (like another command needing the hood), then we can
        // say that we're at the home position of the shooter
        if (!interrupted) {
            RobotContainer.intakeSpool.setCurrentDistance(Parameters.intake.spool.HOME_DISTANCE);
            RobotContainer.intakeSpool.setDesiredDistance(Parameters.intake.spool.UP_DISTANCE);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.intakeSpool.getLSValue();
    }
}
