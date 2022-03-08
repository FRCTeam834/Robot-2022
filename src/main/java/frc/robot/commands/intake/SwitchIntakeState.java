// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwitchIntakeState extends InstantCommand {
    public SwitchIntakeState() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Check the currently set position of the intake
        if (RobotContainer.intakeWinch.getDesiredDistance()
                == Parameters.intake.spool.DOWN_DISTANCE) {

            // Intake is down, we need to put it up
            RobotContainer.intakeWinch.setDesiredDistance(Parameters.intake.spool.UP_DISTANCE);

            // Also shut off the intake
            RobotContainer.intake.stop();
        } else {
            // We must be in the up position or another floating state
            // Intake needs to be put down
            RobotContainer.intakeWinch.setDesiredDistance(Parameters.intake.spool.DOWN_DISTANCE);

            // Also shut off the intake
            RobotContainer.intake.set(Parameters.intake.INTAKE_SPEED);
        }
    }
}
