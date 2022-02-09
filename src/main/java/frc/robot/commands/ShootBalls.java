// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Parameters.indexer;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  public ShootBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.shooter.shoot(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(RobotContainer.shooter.isAtSetPoint())
      {
          RobotContainer.indexer.setMotorSpeed(.5);
      }
      else {
          RobotContainer.indexer.setMotorSpeed(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
