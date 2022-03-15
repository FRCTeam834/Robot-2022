// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class ColorSensorIndexing extends CommandBase {
    /** Creates a new IndexingThings. */
    public ColorSensorIndexing() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!RobotContainer.indexer.hasBall()) {
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
        return false;
    }
}
