// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class DriveForTime extends CommandBase {
    /** Creates a new Drive. */
    Timer timer = new Timer();

    double time = 0;
    double speed = 0;

    public DriveForTime(double speed, double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveTrain);
        this.time = time;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.driveTrain.straightenModules();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.driveTrain.drive(speed, 0, 0, false,false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.zeroVelocities();
        timer.reset();
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
