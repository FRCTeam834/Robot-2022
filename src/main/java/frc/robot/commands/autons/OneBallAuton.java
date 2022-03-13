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
import frc.robot.commands.indexing.IndexForTime;
import frc.robot.commands.intake.IntakeBallsForTime;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.commands.shooting.AutonShot;
import frc.robot.commands.shooting.FenderShot;
import frc.robot.commands.shooting.ManualShoot;
import frc.robot.commands.shooting.PrepareShooterForVision;
import frc.robot.commands.swerve.SpartechsSwerveController;
import frc.robot.commands.swerve.TurnToAngleVision;
import frc.robot.commands.swerve.driving.DriveForTime;
import frc.robot.commands.swerve.driving.SpinForTime;
import frc.robot.subsystems.climber.HomeClimberTubes;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {

    public TwoBallAuton() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new HomeHood(),
                new FenderShot(),
                new WaitCommand(1),
                new IndexForTime(3),
                new DriveForTime(1, 3));
    }
}

