// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class FenderShot extends CommandBase {
    /** Creates a new FenderShot. */
    public FenderShot() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood, RobotContainer.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.hood.setDesiredAngle(75.35);
        RobotContainer.shooter.setDesiredPID(Parameters.shooter.FENDER_SHOT_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // RobotContainer.shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
