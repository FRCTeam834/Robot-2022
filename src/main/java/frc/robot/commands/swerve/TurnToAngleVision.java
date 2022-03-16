// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 1/13/22
 */
package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.Parameters.driveTrain.pid;
import frc.robot.RobotContainer;

import org.photonvision.targeting.PhotonTrackedTarget;

public class TurnToAngleVision extends CommandBase {

    PIDController rotationalPID = new PIDController(3, 0, 0);
    double omega = 0;

    public TurnToAngleVision() {
        rotationalPID.enableContinuousInput(0, 360);
        rotationalPID.setTolerance(.5);
        // Request the subsystem
        addRequirements(RobotContainer.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PhotonTrackedTarget latestResult = RobotContainer.vision.getBestTarget();

        
        if (latestResult == null) {
            omega = 0;
        }
        else {
            omega =
                MathUtil.clamp(
                        Math.toRadians(
                            rotationalPID.calculate(RobotContainer.vision.getBestTarget().getYaw(), 0)),
                        -1,
                        1);
        }

        RobotContainer.driveTrain.drive(
                0 /*RobotContainer.getJoystickValues()[3] * Parameters.driveTrain.maximums.MAX_VELOCITY*/,
                0 /*RobotContainer.getJoystickValues()[2] * Parameters.driveTrain.maximums.MAX_VELOCITY*/,
                -omega,
                false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop all of the modules (basically zero their velocities)
        RobotContainer.driveTrain.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return rotationalPID.atSetpoint();
    }
}
