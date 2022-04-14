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
public class FourBallAuton extends SequentialCommandGroup {

    public FourBallAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory fourBallPart1 = PathPlanner.loadPath("4 Ball Part 1", 3, 1.5);
        PathPlannerTrajectory fourBallPart2 = PathPlanner.loadPath("4 Ball Part 2", 3, 1.5);
        PathPlannerTrajectory fourBallPart3 = PathPlanner.loadPath("4 Ball Part 3", 3, 3);
        addCommands(
            new InstantCommand(
                        () ->
                                RobotContainer.intakeWinch.setCurrentDistance(
                                        Parameters.intake.spool.HOME_DISTANCE)),
            new InstantCommand(
                        () ->
                                RobotContainer.intakeWinch.setDesiredDistance(
                                            Parameters.intake.spool.DOWN_DISTANCE)),
                
                new WaitCommand(.5),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () ->
                                        RobotContainer.driveTrain.resetOdometry(
                                                new Pose2d(
                                                        fourBallPart1
                                                                .getInitialState()
                                                                .poseMeters
                                                                .getTranslation(),
                                                        fourBallPart1.getInitialState()
                                                                .holonomicRotation))),
                        new HomeClimberTubes()),
                new ParallelDeadlineGroup(
                        new FollowPath(fourBallPart1), new IntakeBalls(), new HomeHood()),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new AutoShoot(),
                /*
                new InstantCommand(
                        () ->
                                RobotContainer.driveTrain.resetOdometry(
                                        new Pose2d(
                                                fourBallPart2
                                                        .getInitialState()
                                                        .poseMeters
                                                        .getTranslation(),
                                                fourBallPart2.getInitialState()
                                                        .holonomicRotation))),*/
                new ParallelDeadlineGroup(new FollowPath(fourBallPart2), new IntakeBalls()),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules),
                new ParallelDeadlineGroup(new WaitCommand(1), new IntakeBalls()),
                new InstantCommand(
                        () ->
                                RobotContainer.driveTrain.resetOdometry(
                                        new Pose2d(
                                                fourBallPart3
                                                        .getInitialState()
                                                        .poseMeters
                                                        .getTranslation(),
                                                fourBallPart3.getInitialState()
                                                        .holonomicRotation))),
                new FollowPath(fourBallPart3),
                new AutoShoot(),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
}
