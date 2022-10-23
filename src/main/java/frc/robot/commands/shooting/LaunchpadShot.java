// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class LaunchpadShot extends CommandBase {
    /** Creates a new LaunchpadShot. */
    public LaunchpadShot() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.shooter, RobotContainer.hood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.shooter.setDesiredSpeed(23.675);
        RobotContainer.hood.setDesiredAngle(56.14);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}