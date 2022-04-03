// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.LEDColors;

public class AutoIndex extends CommandBase {

    Timer spitTimer;
    boolean spitting;

    /** Creates a new AutoLoadIntoIndexer. */
    public AutoIndex() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.indexer, RobotContainer.shooter, RobotContainer.hood);

        // Create the spitting timer
        spitTimer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Set that the LEDs should alternate
        RobotContainer.leds.shouldAlternate(true);

        // Default to not spitting
        spitting = false;

        // Idle the shooter
        RobotContainer.shooter.setRPM(2000);

        // Set the hood to the default angle
        RobotContainer.hood.setDesiredAngle(60);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Check if we should be spitting
        if (spitting) {

            // Check if the timer is finished
            if (spitTimer.hasElapsed(.1)) {

                // We're done spitting, stop the indexer and go back to idling
                RobotContainer.indexer.stop();
                RobotContainer.shooter.setRPM(2000);
                RobotContainer.hood.setDesiredAngle(Parameters.hood.IDLE_ANGLE);

                // Set that we're finished spitting
                spitting = false;
            }
        } else {
            // Get the color of the ball
            String color = RobotContainer.indexer.getBallColor();

            // Check if we have a ball
            if (color.equals("None")) {

                // No ball found, we need to continue to load
                RobotContainer.indexer.set(0.05);
                RobotContainer.leds.setSecondaryColor(LEDColors.RED);
            } else if (color.equals("Blue")) {

                // This is a good ball, stop running the indexer, set the color to green
                RobotContainer.indexer.stop();
                RobotContainer.leds.setSecondaryColor(LEDColors.LIME);
            } else {
                // This is a bad ball, set it to be disposed of
                spitting = true;

                // Start the indexer and shooter
                RobotContainer.indexer.set(Parameters.indexer.SPIT_SPEED);
                RobotContainer.shooter.set(.25);
                RobotContainer.hood.setDesiredAngle(105);

                // Reset the timer
                spitTimer.reset();
                spitTimer.start();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // The indexer should default to not running
        RobotContainer.indexer.stop();

        // Also stop the shooter
        RobotContainer.shooter.stop();

        // Set that the LEDs shouldn't alternate (only a primary color)
        RobotContainer.leds.shouldAlternate(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
