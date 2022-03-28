// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.LEDColors;
import frc.robot.utilityClasses.interpolation.ShotParams;

public class ShootBalls extends CommandBase {
    /** Creates a new PrepareShooter. */
    double distance = 0;

    boolean ready = false;
    ShotParams shotParams;
    Timer timer;
    Timer timeSinceLastIndexedBall;

    public ShootBalls() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood, RobotContainer.shooter, RobotContainer.indexer);
        timer = new Timer();
        timeSinceLastIndexedBall = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Reset the timer
        timer.reset();

        // Reset the last indexed ball timer
        timeSinceLastIndexedBall.reset();

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

        if (distance != 0) {
            // Look up the shot parameters for that distance
            shotParams = RobotContainer.interpolatingTable.getShotParam(distance);

            // Set the hood and shooter's desired angles
            RobotContainer.hood.setDesiredAngle(shotParams.getAngle());
            RobotContainer.shooter.setDesiredSpeed(shotParams.getSpeed() + .21375);
        } else {
            RobotContainer.shooter.setRPM(Parameters.shooter.IDLE_RPM);
        }

        if (
        /*RobotContainer.shooter.readyToShoot()*/ timer.hasElapsed(1)) {
            RobotContainer.indexer.set(Parameters.indexer.MOTOR_SPEED);
            ready = true;
            timer.reset();
            timer.start();
            // We're ready to start shooting, turn them green
            RobotContainer.leds.setPrimaryColor(LEDColors.LIME);
        }

        // Check if the time since the last indexed ball needs reset
        if (RobotContainer.indexer.hasBall()) {
            timeSinceLastIndexedBall.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the shooter
        RobotContainer.shooter.stop();
        RobotContainer.indexer.stop();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // Check the if the ball shooting delay has passed and we've started the shooter
        return timeSinceLastIndexedBall.hasElapsed(Parameters.indexer.SHOT_TIME) && ready;
    }
}
