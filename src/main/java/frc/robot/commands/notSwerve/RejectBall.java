// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notSwerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RejectBall extends CommandBase {
    /** Creates a new RejectBall. */
    public RejectBall() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.intake);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!Robot.intake.suckABall().equals(Robot.getOurBallColor())) {
            Robot.intake.spitItOut();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
