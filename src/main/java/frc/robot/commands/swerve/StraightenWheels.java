// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 11/18/21
 */

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class StraightenWheels extends CommandBase {
    /** Creates a new StraightenWheels. */
    public StraightenWheels() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.driveTrain.setDesiredAngles(0, 0, 0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.haltAllModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.driveTrain.areAtDesiredAngles(0, 0, 0, 0);
    }
}
