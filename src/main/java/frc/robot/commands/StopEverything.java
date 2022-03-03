// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class StopEverything extends CommandBase {
  /** Creates a new StopEverything. */
  Timer timer = new Timer();
  public StopEverything() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.driveTrain, RobotContainer.indexer, RobotContainer.intake, RobotContainer.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.start();
      RobotContainer.indexer.stop();
      RobotContainer.driveTrain.haltAllModules();
      RobotContainer.shooter.stop();
      RobotContainer.intake.stop();
      RobotContainer.hood.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      timer.reset();
      timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(.2);
  }
}
