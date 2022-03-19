// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class IntakeBallsForTime extends CommandBase {
    /** Creates a new IntakeBalls. */
    Timer timer = new Timer();

    double time = 0;

    public IntakeBallsForTime(double time) {
        addRequirements(RobotContainer.intake);
        this.time = time;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.intake.set(.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
