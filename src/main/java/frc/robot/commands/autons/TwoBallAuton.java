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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Parameters.driveTrain;
import frc.robot.commands.StopEverything;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.indexing.ColorSensorIndexing;
import frc.robot.commands.indexing.IndexStupid;
import frc.robot.commands.intake.IntakeBalls;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.commands.shooting.DumbShoot;
import frc.robot.commands.shooting.PrepareShooter;
import frc.robot.commands.swerve.SpartechsSwerveController;
import frc.robot.commands.swerve.TurnToAngleVision;
import frc.robot.commands.swerve.driving.DriveForTime;
import frc.robot.commands.swerve.driving.SpinForTime;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {

    public TwoBallAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(new IntakeBalls(2), new DriveForTime(-1, 2), new HomeHood()),
                new SpinForTime(2, 1.5),
                new ParallelCommandGroup(new TurnToAngleVision(), new PrepareShooter()).withInterrupt(RobotContainer.shooter::readyToShoot),
                new IndexStupid(2),
                new StopEverything());
    }
}

