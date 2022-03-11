// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DumbHood extends CommandBase {
  /** Creates a new DumbHood. */
  public DumbHood() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.hood.set(-RobotContainer.xbox.getAxisType(2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.hood.set(-RobotContainer.xbox.getAxisType(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
