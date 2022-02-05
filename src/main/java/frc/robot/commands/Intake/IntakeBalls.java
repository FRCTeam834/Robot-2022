// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// Intakes as many balls as possible, ie. if there is one ball in robot it intakes one ball, if theres 2 it wont intake any
public class IntakeBalls extends CommandBase {
    boolean increment = false;
  /** Creates a new IntakeBalls. */
  public IntakeBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.intake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If there isn't a ball in the top sensor turn on bottom shooter wheel
    if(!RobotContainer.shooter.getTopSensor()){
        RobotContainer.shooter.setBottomMotorSpeed(Parameters.shooter.motor.BOTTOM_SPEED);
    }

        // If there is a ball in the bottom shooter
        if(RobotContainer.shooter.getBottomSensor()){
            // Has the sensor changed, as in was the bottom sensor false before
            if(RobotContainer.shooter.getSensorChanged()){
                // sensorChanged is reset
                RobotContainer.shooter.setSensorChanged(false);
                // A ball is added to the count
                RobotContainer.shooter.addBallCount(1);
            }
        } else {
            // If the if statement that checks if this is true is being evaluated, it means that the sensor will have
            // Changed, that is why it is set to true here even if the sensor hasn't changed at this point
            RobotContainer.shooter.setSensorChanged(true);
            
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
