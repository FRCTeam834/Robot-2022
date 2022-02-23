// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class EmptyBallz extends CommandBase {

    // If we need to control the shooter or not
    boolean controlShooter;

    // Create a new timer
    Timer timer;

    /** Creates a new EmptyBallz */
    public EmptyBallz(boolean controlShooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        if (controlShooter) {
            addRequirements(RobotContainer.shooter, RobotContainer.indexer);
        } else {
            // No need for taking over the shooter unless needed
            addRequirements(RobotContainer.indexer);
        }

        // Save if we need to control the shooter
        this.controlShooter = controlShooter;

        // Create the timer
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (controlShooter) {
            RobotContainer.shooter.setDesiredSpeed(Parameters.shooter.SHOT_SPEED);
        }

        // Reset the timer
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set the indexer to run when the shooter is ready
        if (RobotContainer.shooter.isAtSetPoint()) {
            RobotContainer.indexer.setMotorSpeed(Parameters.indexer.MOTOR_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Shut off the shooter only if we need to
        if (controlShooter) {
            RobotContainer.shooter.stop();
        }

        RobotContainer.indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Parameters.indexer.SHOT_TIME);
    }
}
