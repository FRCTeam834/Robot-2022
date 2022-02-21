// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveUntilAngle extends CommandBase {

    // The speed to move the robot at
    double tipSpeed;

    // The angle to tip the robot to
    double tipAngle;

    /** Creates a new DriveUntilAngle. */
    public DriveUntilAngle(double speed, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain, RobotContainer.navX);

        // Save the tip speed and angle
        tipSpeed = speed;
        tipAngle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Start moving the drivetrain forward slowly
        RobotContainer.driveTrain.drive(tipSpeed, 0, 0, false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the drivetrain once we're done
        RobotContainer.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (RobotContainer.navX.getPitch() >= tipAngle);
    }
}
