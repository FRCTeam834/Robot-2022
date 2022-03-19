// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.swerve.FollowPath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTesting extends SequentialCommandGroup {

    public PathPlannerTesting() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", .5, .25);
        addCommands(
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new InstantCommand(
                        () ->
                                RobotContainer.driveTrain.resetOdometry(
                                        new Pose2d(
                                                examplePath
                                                        .getInitialState()
                                                        .poseMeters
                                                        .getTranslation(),
                                                examplePath.getInitialState().holonomicRotation))),
                new FollowPath(examplePath),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
}
