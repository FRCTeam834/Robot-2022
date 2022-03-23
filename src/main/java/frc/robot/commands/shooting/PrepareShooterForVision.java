// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.interpolation.ShotParams;

public class PrepareShooterForVision extends CommandBase {
    /** Creates a new PrepareShooter. */
    double distance = 0;
    boolean ready = false;
    ShotParams shotParams;
    Timer timer;

    public PrepareShooterForVision() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.hood, RobotContainer.shooter);
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.stop();

        // Clear the distance average
        RobotContainer.vision.flushDistAvg();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Compute the distance from the target using the camera
        distance = RobotContainer.vision.getDistanceToGoalInches();

        // Look up the shot parameters for that distance
        shotParams = RobotContainer.interpolatingTable.getShotParam(distance);

        // Set the hood and shooter's desired angles
        RobotContainer.hood.setDesiredAngle(shotParams.getAngle());
        RobotContainer.shooter.setDesiredPID(shotParams.getSpeed());

        // else {
        //   RobotContainer.hood.setCurrentAngle(Parameters.shooter.FENDER_HOOD_ANGLE);
        // RobotContainer.shooter.setDesiredPID(Parameters.shooter.FENDER_SHOT_SPEED);
        // }
        if (RobotContainer.shooter.readyToShoot()) {
            RobotContainer.indexer.set(Parameters.indexer.MOTOR_SPEED);
            ready = true;
            timer.reset();
            timer.start();
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
        return timer.hasElapsed(3) && ready;
    }
}
