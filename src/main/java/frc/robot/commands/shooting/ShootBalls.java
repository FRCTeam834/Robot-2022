// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.LEDColors;
import frc.robot.utilityClasses.interpolation.ShotParams;

public class ShootBalls extends CommandBase {
    /** Creates a new PrepareShooter. */
    double distance = 0;

    boolean feeding = false;
    ShotParams shotParams;
    Timer timeSinceLastIndexedBall;
    Timer intakePulseTimer;

    public ShootBalls() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(
                RobotContainer.hood,
                RobotContainer.shooter,
                RobotContainer.indexer,
                RobotContainer.intake);
        timeSinceLastIndexedBall = new Timer();
        intakePulseTimer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Remove the default command
        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.indexer, null);

        // Reset the last indexed ball timer
        timeSinceLastIndexedBall.reset();

        // Turn on the intake (will shift balls down into the pocket if they're stuck on the front
        // bumper)
        RobotContainer.intake.set(Parameters.intake.INTAKE_SPEED);
        intakePulseTimer.reset();

        // Clear the distance average
        RobotContainer.vision.flushDistAvg();

        // Red LEDs
        RobotContainer.leds.setPrimaryColor(LEDColors.STROBE_RED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Compute the distance from the target using the camera
        distance = RobotContainer.vision.getDistanceToGoalInches();

        // If the distance isn't zero, we have a target
        if (distance != 0) {

            // Look up the shot parameters for that distance
            shotParams = RobotContainer.interpolatingTable.getShotParam(distance);

            // Set the hood and shooter's desired angles
            RobotContainer.hood.setDesiredAngle(shotParams.getAngle());
            RobotContainer.shooter.setDesiredSpeed(shotParams.getSpeed() + .21375);
        } else {
            RobotContainer.shooter.setRPM(Parameters.shooter.IDLE_RPM);
        }

        // If everything is ready, we can start the indexer/LEDs
        if (RobotContainer.shooter.isReady()
                && RobotContainer.hood.isAtDesiredAngle()
                && RobotContainer.vision.isLinedUp()) {

            // Start the indexer
            RobotContainer.indexer.set(Parameters.indexer.FEED_DUTY);

            // Reset the timer for feeding
            // We shouldn't do this, but it's needed to prevent the command from exiting prematurely
            timeSinceLastIndexedBall.reset();

            // Set that the shooter is ready for feeding
            feeding = true;

            // We're ready to start shooting, turn them green
            RobotContainer.leds.setPrimaryColor(LEDColors.LIME);
        }

        // Check if the time since the last indexed ball needs reset
        if (RobotContainer.indexer.hasBall()) {
            timeSinceLastIndexedBall.reset();
        }

        // Turn off the intake if we're done pulsing
        if (intakePulseTimer.hasElapsed(Parameters.intake.PULSE_TIME)) {
            RobotContainer.intake.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the shooter
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();

        // Stop indexing the balls
        RobotContainer.autoIndex = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Check the if the ball shooting delay has passed and we've started the shooter
        return timeSinceLastIndexedBall.hasElapsed(Parameters.indexer.SHOT_TIME) && feeding;
    }
}
