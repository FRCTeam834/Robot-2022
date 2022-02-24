// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.interpolation.ShotParams;

public class PrepareShooter extends CommandBase {
  /** Creates a new PrepareShooter. */
  public PrepareShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood, RobotContainer.shooter);

  }

// Called when the command is initially scheduled.
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {

    // Compute the distance from the target using the camera
    double distance = RobotContainer.vision.getDistanceToGoal();

    // Look up the shot parameters for that distance
    ShotParams shotParams = RobotContainer.interpolatingTable.getShotParam(distance);

    // Set the hood and shooter's desired angles
    RobotContainer.hood.setDesiredAngle(shotParams.getAngle());
    RobotContainer.shooter.setDesiredSpeed(shotParams.getSpeed());
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {

    // Stop the shooter
    RobotContainer.shooter.stop();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
        return false;
    }

}

