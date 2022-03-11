// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class dumbintakeandshoot extends CommandBase {
  /** Creates a new dumbintakeandshoot. */
  public dumbintakeandshoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.indexer, RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.intake.set(.5);
      RobotContainer.indexer.set(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.indexer.stop();
      RobotContainer.intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
