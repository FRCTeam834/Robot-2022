// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;


public class ShootBall extends CommandBase {

    Color ourTeam;
  /** Creates a new ShootBall. */
  public ShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.spinShooterMotor(Parameters.shooter.SPEED);
// TODO set up way to get team color
    if(1 == 1){
        
        ourTeam = Color.kRed;
    }

    else{
        ourTeam = Color.kBlue;
    }
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  // If ball is the wrong color, squib the shot if not shoot it properly
  @Override
  public void execute() {
      if(RobotContainer.shooter.getClosestColor().equals(ourTeam)) {
        RobotContainer.shooter.spinShooterMotor(Parameters.shooter.SPEED);
      }

      else {
        RobotContainer.shooter.spinShooterMotor(Parameters.shooter.LOW_SPEED);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.shooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}