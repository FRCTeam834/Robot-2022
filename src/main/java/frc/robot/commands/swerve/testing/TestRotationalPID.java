// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.testing;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class TestRotationalPID extends CommandBase {
    /** Creates a new RotationalPIDTesting. */
    public TestRotationalPID() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Set that the new angle of the robot
        RobotContainer.driveTrain.rotationPID.setGoal(
                Units.degreesToRadians(RobotContainer.navX.getYaw() + 90));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Calculate the desired rotation speed
        double rotationSpeed =
                -RobotContainer.driveTrain.rotationPID.calculate(
                        Units.degreesToRadians(RobotContainer.navX.getYaw()));

        // Set that the drivetrain should move there
        RobotContainer.driveTrain.drive(0, 0, rotationSpeed, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Make sure that the robot isn't left moving
        RobotContainer.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
