// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

import org.photonvision.PhotonCamera;

public class TurnToVision extends CommandBase {

    // Declaring variables
    final double angular_P = 0.0;
    final double angular_D = 0.0;
    PIDController angularPid = new PIDController(angular_P, 0, angular_D);
    // angularPid.setTolerance(5, 10);

    PhotonCamera SparTechsCamera = new PhotonCamera("camera");

    /** Creates a new TurnToVision. */
    public TurnToVision() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        var result = SparTechsCamera.getLatestResult();

        if (result.hasTargets()) {
            Robot.driveTrain.drive(
                    0.0,
                    0.0,
                    MathUtil.clamp(
                            angularPid.calculate(result.getBestTarget().getYaw(), 0), -0.5, 0.5),
                    false);
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.driveTrain.drive(0.0, 0.0, 0.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return angularPid.atSetpoint();
    }
}
