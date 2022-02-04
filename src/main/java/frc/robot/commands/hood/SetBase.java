// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;
import frc.robot.subsystems.Hood;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetBase extends CommandBase {
  /** Creates a new SetBase. */
  public SetBase() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(RobotContainer.hood);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.hood.hoodBackwards();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
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
