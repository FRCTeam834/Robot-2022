// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.Parameters.shooter;

public class AutonShot extends CommandBase {
    /** Creates a new FenderShot. */
    public AutonShot() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood, RobotContainer.shooter);
    }

<<<<<<< HEAD
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
              //RobotContainer.hood.setCurrentAngle(23.59);
               //RobotContainer.shooter.setDesiredPID(55.079);
  }
=======
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.hood.setCurrentAngle(23.59);
        RobotContainer.shooter.setDesiredPID(55.079);
    }
>>>>>>> 98a9a0007830f051a05c5f489e5e47f24c01f09d

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
