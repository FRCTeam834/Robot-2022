// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class StopClimb extends CommandBase {
    Timer timer = new Timer();
  /** Creates a new StopClimb. */
  public StopClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climbers2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.start();
      RobotContainer.climbers2.rightLift.set(0);
      RobotContainer.climbers2.leftLift.set(0);
      RobotContainer.climbers2.leftTilt.set(0);
    RobotContainer.climbers2.rightTilt.set(0);
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
    return timer.hasElapsed(1);
  }
}
