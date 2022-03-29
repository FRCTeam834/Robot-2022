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
import frc.robot.RobotContainer;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.intake.ColorSensorIntaking;
import frc.robot.commands.shooting.PrepareShooterForVision;
import frc.robot.commands.swerve.FollowPath;
import frc.robot.commands.swerve.TurnToAngleVision;
import frc.robot.subsystems.climber.HomeClimberTubes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuton extends SequentialCommandGroup {

    public FourBallAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        PathPlannerTrajectory fourBallPart1 = PathPlanner.loadPath("Four Ball Part 1", 4, 3);
        PathPlannerTrajectory fourBallPart2 = PathPlanner.loadPath("Four Ball Part 2", 4, 3);
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
                                                        fourBallPart1
                                                                .getInitialState()
                                                                .poseMeters
                                                                .getTranslation(),
                                                        fourBallPart2.getInitialState()
                                                                .holonomicRotation))),
                        new HomeClimberTubes(),
                        new InstantCommand(
                                () ->
                                        RobotContainer.intakeWinch.setDesiredDistance(
                                                Parameters.intake.spool.DOWN_DISTANCE))),
                new ParallelDeadlineGroup(
                        new FollowPath(fourBallPart1), new ColorSensorIntaking(), new HomeHood()),
                new ParallelRaceGroup(
                                new TurnToAngleVision(true, false), new PrepareShooterForVision())
                        .withTimeout(3),
                new ParallelDeadlineGroup(new FollowPath(fourBallPart2), new ColorSensorIntaking()),
                new ParallelRaceGroup(
                        new TurnToAngleVision(true, false), new PrepareShooterForVision()),
                new InstantCommand(RobotContainer.driveTrain::haltAllModules));
    }
}
