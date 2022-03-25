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

import frc.robot.RobotContainer;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.intake.ColorSensorIntaking;
import frc.robot.commands.shooting.IdleShooter;
import frc.robot.commands.shooting.PrepareShooterForVision;
import frc.robot.commands.swerve.FollowPath;
import frc.robot.commands.swerve.TurnToAngleVision;
import frc.robot.commands.swerve.driving.DriveForTime;
import frc.robot.subsystems.climber.HomeClimberTubes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTesting extends SequentialCommandGroup {

    public PathPlannerTesting() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Two Ball", 4, 2);
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () ->
                                        RobotContainer.driveTrain.resetOdometry(
                                                new Pose2d(
                                                        examplePath
                                                                .getInitialState()
                                                                .poseMeters
                                                                .getTranslation(),
                                                        examplePath.getInitialState()
                                                                .holonomicRotation))),
                        new HomeClimberTubes()),
                new ParallelDeadlineGroup(
                        new FollowPath(examplePath),
                        new ColorSensorIntaking(),
                        new HomeHood(),
                        new IdleShooter()),
                new ParallelRaceGroup(
                                new TurnToAngleVision(true, false), new PrepareShooterForVision())
                        .withTimeout(3),
                new ParallelRaceGroup(new DriveForTime(2, 1), new ColorSensorIntaking()),
                new ParallelRaceGroup(
                        new ColorSensorIntaking(),
                        new TurnToAngleVision(true, false),
                        new PrepareShooterForVision()),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
}
