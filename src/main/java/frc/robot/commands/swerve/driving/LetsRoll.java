/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/8/20
 */
package frc.robot.commands.swerve.driving;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class LetsRoll extends CommandBase {

    public LetsRoll() {
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
        double leftX, rightX, rightY;

        // Get all of the current joystick inputs
        if (true) {
            leftX = RobotContainer.constrainJoystick(RobotContainer.leftJoystick.getX());
            rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
            rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());
        }
        else {
            leftX = RobotContainer.constrainJoystick(RobotContainer.quadController.getRawAxis(0));
            rightX = RobotContainer.constrainJoystick(RobotContainer.quadController.getRawAxis(2));
            rightY = RobotContainer.constrainJoystick(RobotContainer.quadController.getRawAxis(3));
        }

        // If any of the sticks are out of range, then we need to move. Otherwise, lock up the
        // drivetrain (if specified) or just halt the modules
        if (leftX != 0 || rightX != 0 || rightY != 0) {

            // Move the drivetrain with the desired values (left right values are flipped from the
            // logical
            // way, thanks WPI)
            RobotContainer.driveTrain.drive(
                    (rightY * Parameters.driver.maxModVelocity),
                    (rightX * Parameters.driver.maxModVelocity),
                    Math.toRadians(leftX * RobotContainer.turnRate),
                    RobotContainer.fieldCentric);
        } else if (Parameters.driver.lockemUp) {
            RobotContainer.driveTrain.lockemUp();
        } else {
            RobotContainer.driveTrain.stopModules();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
