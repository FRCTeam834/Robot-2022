// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class FenderShot extends CommandBase {
    /** Creates a new FenderShot. */
    public FenderShot() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood, RobotContainer.shooter, RobotContainer.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.shooter.set(2800.0 / 5280.0);
        RobotContainer.hood.setDesiredAngle(76);

        if (RobotContainer.shooter.isReady() && RobotContainer.hood.isAtDesiredAngle()) {
            RobotContainer.indexer.set(0.5);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !RobotContainer.indexer.hasBall();
    }
}
