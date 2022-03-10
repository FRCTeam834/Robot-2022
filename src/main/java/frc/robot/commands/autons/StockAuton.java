// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.StopEverything;
import frc.robot.commands.swerve.SpartechsSwerveController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StockAuton extends SequentialCommandGroup {

    public StockAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory threeBall = PathPlanner.loadPath("Debug", 1, .5);
        addCommands(
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new InstantCommand(()-> RobotContainer.driveTrain.setDesiredAngles(0, 0, 0, 0)),
                new InstantCommand(RobotContainer.driveTrain::resetAllPIDControllers),
                new InstantCommand(
                        () ->
                                RobotContainer.driveTrain.resetOdometry(new Pose2d(new Translation2d(7.63,2.84), new Rotation2d(-109.36)))),
                RobotContainer.driveTrain.getPPSwerveContollerCommand(threeBall),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new StopEverything());
    }
}

