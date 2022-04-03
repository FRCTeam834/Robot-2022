// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.swerve.FollowPath;
import frc.robot.subsystems.climber.HomeClimberTubes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {

    public TestAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory testPart1 = PathPlanner.loadPath("Test Part 1", 4, 3);
        PathPlannerTrajectory testPart2 = PathPlanner.loadPath("Test Part 2", 4, 3);
        addCommands(
                new InstantCommand(() -> RobotContainer.driveTrain.resetOdometry(new Pose2d(testPart1.getInitialState().poseMeters.getTranslation(), testPart1.getInitialState().holonomicRotation))),
                new FollowPath(testPart1),
                new FollowPath(testPart2),
                new InstantCommand(() -> RobotContainer.driveTrain.haltAllModules())
                );

    }
}
