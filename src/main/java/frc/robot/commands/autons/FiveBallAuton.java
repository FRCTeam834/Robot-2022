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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.commands.swerve.FollowPath;
import frc.robot.subsystems.climber.HomeClimberTubes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuton extends SequentialCommandGroup {

    public FiveBallAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory fiveBallPart1 = PathPlanner.loadPath("5 Ball Part 1", 4, 3);
        PathPlannerTrajectory fiveBallPart2 = PathPlanner.loadPath("5 Ball Part 2", 4, 3);
        PathPlannerTrajectory fiveBallPart3 = PathPlanner.loadPath("5 Ball Part 3", 4, 3);
        PathPlannerTrajectory fiveBallPart4 = PathPlanner.loadPath("5 Ball Part 4", 4, 3);
        addCommands(
                new InstantCommand(
                        () ->
                                RobotContainer.intakeWinch.setCurrentDistance(
                                        Parameters.intake.spool.HOME_DISTANCE)),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () ->
                                        RobotContainer.driveTrain.resetOdometry(
                                                new Pose2d(
                                                        fiveBallPart1
                                                                .getInitialState()
                                                                .poseMeters
                                                                .getTranslation(),
                                                        fiveBallPart1.getInitialState()
                                                                .holonomicRotation))),
                        new HomeClimberTubes(),
                        new InstantCommand(
                                () ->
                                        RobotContainer.intakeWinch.setDesiredDistance(
                                                Parameters.intake.spool.DOWN_DISTANCE))),
                new ParallelDeadlineGroup(
                        new FollowPath(fiveBallPart1), new IntakeBalls(), new HomeHood()),
                new AutoShoot().withTimeout(2),
                new ParallelDeadlineGroup(new FollowPath(fiveBallPart2), new IntakeBalls()),
                new ParallelDeadlineGroup(new WaitCommand(2), new IntakeBalls()),
                new FollowPath(fiveBallPart3),
                new AutoShoot().withTimeout(3),
                new FollowPath(fiveBallPart4),
                new AutoShoot().withTimeout(3),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
}
