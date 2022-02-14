// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {
    /** Creates a new ShootOneBall. */
    public Shoot() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.shooter, RobotContainer.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.shooter.set(Parameters.shooter.SHOT_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.shooter.isAtSetPoint()) {
            RobotContainer.indexer.setMotorSpeed(Parameters.indexer.MOTOR_SPEED);
        } else {
            RobotContainer.indexer.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
