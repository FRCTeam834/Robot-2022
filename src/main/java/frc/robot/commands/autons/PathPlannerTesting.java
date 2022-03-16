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

import frc.robot.RobotContainer;
<<<<<<< HEAD

=======
>>>>>>> f7283b2ec6b7a7d4008a0249270fe16b834f4c9d

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTesting extends SequentialCommandGroup {
<<<<<<< HEAD

    public PathPlannerTesting() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Straight Test", 8, 5);
        addCommands(
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new InstantCommand(
                        () ->
                                RobotContainer.driveTrain.resetOdometry(new Pose2d(examplePath.getInitialState().poseMeters.getTranslation(), examplePath.getInitialState().holonomicRotation))),
                new FollowPath(examplePath),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
=======
  /** Creates a new PathPlannerTesting. */
  public PathPlannerTesting() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("SparTechs Path", 8, 5);
    addCommands(
        new InstantCommand(RobotContainer.driveTrain::haltAllModules),
        new InstantCommand(
                () ->
                        RobotContainer.driveTrain.resetOdometry(
                                examplePath.getInitialPose())),
        new FollowPath(examplePath),
        new InstantCommand(RobotContainer.driveTrain::lockemUp)
    );
  }
>>>>>>> f7283b2ec6b7a7d4008a0249270fe16b834f4c9d
}
