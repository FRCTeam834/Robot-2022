// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.commands.indexing.AutoIndex;

public class IntakeBalls extends CommandBase {
    /** Creates a new IntakeBalls. */
    public IntakeBalls() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Run the intake at the regular speed
        RobotContainer.intake.set(Parameters.intake.INTAKE_SPEED);

        // Tell the indexer to default to autoindexing
        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.indexer, new AutoIndex());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // The indexer is done, so stop it
        RobotContainer.intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
