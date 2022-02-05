// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class IntakeOneBall extends CommandBase {

    boolean done = false;

    /** Creates a new IntakeBall. */
    public IntakeOneBall() {
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
        if (!RobotContainer.shooter.getTopSensor()) {
            RobotContainer.shooter.setBottomMotorSpeed(Parameters.shooter.motor.BOTTOM_SPEED);
        }
        /*
          If the bottom sensor is tripped, check if

        */
        if (!(RobotContainer.shooter.getBottomSensor() && RobotContainer.shooter.getTopSensor())) {

            if (RobotContainer.shooter.getBottomSensor()) {
                if (RobotContainer.shooter.getSensorChanged()) {
                    RobotContainer.shooter.setSensorChanged(false);
                    RobotContainer.shooter.addBallCount(1);
                }

            } else {
                RobotContainer.shooter.setSensorChanged(true);
            }
        } else {
            // Command should end
            done = true;
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
