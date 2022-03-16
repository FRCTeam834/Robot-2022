// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WaitForShooter extends CommandBase {

    Timer timer = new Timer();

    /** Creates a new WaitForShooter. */
    public WaitForShooter() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset the time of the timer
        timer.reset();
        timer.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(RobotContainer.shooter.readyToShoot()) {
            RobotContainer.indexer.set(.5);
            timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.indexer.stop();
        RobotContainer.shooter.stop();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }
}
