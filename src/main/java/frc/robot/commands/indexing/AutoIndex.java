// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.Robot;
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
        // RobotContainer.leds.shouldAlternate(true);

        // Default to not spitting
        spitting = false;

        // Idle the shooter
        RobotContainer.shooter.setRPM(Parameters.shooter.IDLE_RPM);

        // Set the hood to the default angle
        RobotContainer.hood.setDesiredAngle(Parameters.hood.IDLE_ANGLE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Check if we should be spitting
        /*if (spitting) {

            // Check if the timer is finished
            if (spitTimer.hasElapsed(Parameters.indexer.SPIT_TIME)) {

                // We're done spitting, stop the indexer and go back to idling
                RobotContainer.indexer.stop();
                RobotContainer.shooter.setRPM(Parameters.shooter.IDLE_RPM);
                RobotContainer.hood.setDesiredAngle(Parameters.hood.IDLE_ANGLE);

                // Set that we're finished spitting
                spitting = false;
            }
            if (RobotContainer.shooter.isReady() && RobotContainer.hood.isAtDesiredAngle()) {
                RobotContainer.indexer.set(Parameters.indexer.SPIT_DUTY);
                spitTimer.start();
            }
        } else {*/
            // Get the color of the ball
            String color = RobotContainer.indexer.getBallColor();

            // Check if we have a ball
            if (color.equals("None")) {

                // No ball found, we need to continue to load
                RobotContainer.indexer.set(Parameters.indexer.LOAD_DUTY);
                RobotContainer.leds.setSecondaryColor(LEDColors.RED);
            } else {

                // This is a good ball, stop running the indexer, set the color to green
                RobotContainer.indexer.stop();
                RobotContainer.leds.setSecondaryColor(LEDColors.LIME);
            } /*else {
                // This is a bad ball, set it to be disposed of
                spitting = true;

                // Start the shooter, set the angle
                RobotContainer.shooter.setDesiredSpeed(Parameters.shooter.SPIT_SPEED);
                RobotContainer.hood.setDesiredAngle(Parameters.hood.SPIT_ANGLE);

                // Make sure that the indexer isn't running
                RobotContainer.indexer.stop();

                // Set the LEDs to be strobing red
                RobotContainer.leds.setSecondaryColor(LEDColors.RED);

                // Reset the timer, but don't start it. We'll start it once the hood has reached the
                // correct angle
                spitTimer.reset();
            }*/
        //}
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // The indexer should default to not running
        RobotContainer.indexer.stop();

        // Also stop the shooter
        RobotContainer.shooter.stop();

        // Set that the LEDs shouldn't alternate (only a primary color)
        // RobotContainer.leds.shouldAlternate(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
