// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.LEDColors;

public class AutoLoadIntoIndexer extends CommandBase {
    /** Creates a new AutoLoadIntoIndexer. */
    public AutoLoadIntoIndexer() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Set that the LEDs should alternate
        RobotContainer.leds.shouldAlternate(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!RobotContainer.indexer.hasBall()) {
            RobotContainer.indexer.set(0.1);
            RobotContainer.leds.setSecondaryColor(LEDColors.RED);
        } else {
            RobotContainer.indexer.stop();
            RobotContainer.leds.setSecondaryColor(LEDColors.LIME);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // The indexer should default to not running
        RobotContainer.indexer.stop();

        // Set that the LEDs shouldn't alternate (only a primary color)
        RobotContainer.leds.shouldAlternate(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
