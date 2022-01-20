// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notSwerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ColorSensor;
import frc.robot.Robot;

public class RejectBall extends CommandBase {
  /** Creates a new RejectBall. */

  Color color;

  public RejectBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.colorSensor, RobotContainer.intake);
  }

  // You thought that this comment would explain what initialize does, but it was me, DIO!
  @Override
  public void initialize() {
    //spin the intake motors

    //Todo set value for ball color
    if(Robot.ballRejectColor.getBoolean(false)){
      color = Color.kRed;
      RobotContainer.intake.setForward();
    }
    else{
      color = Color.kBlue;
      RobotContainer.intake.setReverse();
    }
    RobotContainer.intake.setForward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.colorSensor.getReading() == color) {
      RobotContainer.intake.setReverse();
    }
    else {
      RobotContainer.intake.setForward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
