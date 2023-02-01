// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCone extends CommandBase {
  /** Creates a new IntakeCone. */
  public final LinearFilter filter;
  private double a;
  public IntakeCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    filter = LinearFilter.movingAverage(12);
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    filter.reset();
    filter.calculate(-100000);
    RobotContainer.intake.startMotorBackward();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    a = filter.calculate(RobotContainer.intake.intakeEncoder.getVelocity());
    DriverStation.reportWarning(String.format("%f", a), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(a) < 20;
  }
}