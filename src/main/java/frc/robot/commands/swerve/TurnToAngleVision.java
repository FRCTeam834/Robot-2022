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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
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

        double rightX = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getX());
        double rightY = RobotContainer.constrainJoystick(RobotContainer.rightJoystick.getY());

        if (latestResult == null) {
            if (DriverStation.isFMSAttached()) {
                Pose2d robotPose = RobotContainer.vision.getLastPoseFromVision();
                double facingInRadians = RobotContainer.navX.getRotation2d().getRadians();
                double x = robotPose.getX();
                double y = robotPose.getY();
                double targetRadians =
                        Math.atan2(Parameters.vision.GOAL_X - y, Parameters.vision.GOAL_Y - x);
                double closestAngle =
                        (targetRadians - facingInRadians + Math.toRadians(540))
                                        % Math.toRadians(360)
                                - Math.toRadians(180);
                omega = -Math.signum(closestAngle);
            } else {
                omega = 0;
            }
        } else {
            omega =
                    MathUtil.clamp(
                            Math.toRadians(
                                    rotationalPID.calculate(
                                            latestResult.getYaw(), Parameters.vision.YAW_OFFSET)),
                            -1,
                            1);
        }

        RobotContainer.driveTrain.drive(
                (-rightY * Parameters.driver.maxModVelocity),
                (-rightX * Parameters.driver.maxModVelocity),
                omega,
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
