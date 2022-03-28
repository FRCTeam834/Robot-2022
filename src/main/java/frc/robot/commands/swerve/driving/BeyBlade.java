// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class BeyBlade extends CommandBase {
    /** Creates a new BeyBlade. */
    public BeyBlade() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Create variables for storing movement values
        double rightX, rightY;

        // Get all of the current joystick inputs
        rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
        rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());

        // If any of the sticks are out of range, then we need to move. Otherwise, lock up the
        // drivetrain (if specified) or just halt the modules
        if (rightX != 0 || rightY != 0) {

            // Move the drivetrain with the desired values (left right values are flipped from the
            // logical way, thanks WPI)
            RobotContainer.driveTrain.drive(
                    (-rightY * Parameters.driver.maxModVelocity),
                    (-rightX * Parameters.driver.maxModVelocity),
                    Math.toRadians(180 * 4),
                    Parameters.driver.fieldCentric,
                    Parameters.driver.tipProtection.USING_TIP_PROTECTION);
        } else if (Parameters.driver.lockemUp) {
            RobotContainer.driveTrain.lockemUp();
        } else {
            RobotContainer.driveTrain.zeroVelocities();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.zeroVelocities();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
