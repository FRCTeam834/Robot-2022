// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 3/9/21
 */
package frc.robot.commands.swerve.test;

// Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class TestModulePID extends CommandBase {
    /** Creates a new TestPID. */
    public TestModulePID() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Wait for half a second
        Timer.delay(.5);

        // Move each of the wheels to 0
        RobotContainer.driveTrain.setDesiredAngles(0, 0, 0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // The top right button
        if (RobotContainer.leftJoystick.getRawButton(11)) {
            RobotContainer.driveTrain.frontRight.setDesiredAngle(45);
            Timer.delay(1);
            RobotContainer.driveTrain.frontRight.setDesiredAngle(-45);
            Timer.delay(1);
        }

        // The top left button
        else if (RobotContainer.leftJoystick.getRawButton(6)) {
            RobotContainer.driveTrain.frontLeft.setDesiredAngle(45);
            Timer.delay(1);
            RobotContainer.driveTrain.frontLeft.setDesiredAngle(-45);
            Timer.delay(1);
        }

        // The bottom left button
        else if (RobotContainer.leftJoystick.getRawButton(7)) {
            RobotContainer.driveTrain.backLeft.setDesiredAngle(45);
            Timer.delay(1);
            RobotContainer.driveTrain.backLeft.setDesiredAngle(-45);
            Timer.delay(1);
        }

        // The bottom right button
        else if (RobotContainer.leftJoystick.getRawButton(10)) {
            RobotContainer.driveTrain.backRight.setDesiredAngle(45);
            Timer.delay(1);
            RobotContainer.driveTrain.backRight.setDesiredAngle(-45);
            Timer.delay(1);
        }

        // The top right button
        else if (RobotContainer.rightJoystick.getRawButton(11)) {
            RobotContainer.driveTrain.frontRight.setDesiredVelocity(2);
            Timer.delay(3);
            RobotContainer.driveTrain.frontRight.setDesiredVelocity(-2);
            Timer.delay(3);
        }

        // The top left button
        else if (RobotContainer.rightJoystick.getRawButton(6)) {
            RobotContainer.driveTrain.frontLeft.setDesiredVelocity(2);
            Timer.delay(3);
            RobotContainer.driveTrain.frontLeft.setDesiredVelocity(-2);
            Timer.delay(3);
        }

        // The bottom left button
        else if (RobotContainer.rightJoystick.getRawButton(7)) {
            RobotContainer.driveTrain.backLeft.setDesiredVelocity(2);
            Timer.delay(3);
            RobotContainer.driveTrain.backLeft.setDesiredVelocity(-2);
            Timer.delay(3);
        }

        // The bottom right button
        else if (RobotContainer.rightJoystick.getRawButton(10)) {
            RobotContainer.driveTrain.backRight.setDesiredVelocity(2);
            Timer.delay(3);
            RobotContainer.driveTrain.backRight.setDesiredVelocity(-2);
            Timer.delay(3);
        } else {

            // Halt the motors, then move to the zeros
            RobotContainer.driveTrain.setDesiredVelocities(0, 0, 0, 0);
            RobotContainer.driveTrain.setDesiredAngles(0, 0, 0, 0);
        }

        // Publishes velocity and angle to network tables
        RobotContainer.driveTrain.publishPerformanceData();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.leftJoystick.getTrigger();
    }
}
