// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.StopEverything;
import frc.robot.commands.climber.StopClimb;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.indexing.IndexForTime;
import frc.robot.commands.intake.ColorSensorIntaking;
import frc.robot.commands.intake.HomeIntake;
import frc.robot.commands.shooting.FenderShot;
import frc.robot.commands.shooting.IdleShooter;
import frc.robot.commands.shooting.PrepareShooterForVision;
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
                new ParallelCommandGroup(new HomeHood(), new HomeClimberTubes(), new HomeIntake()),
                new ParallelRaceGroup(new DriveForTime(1,3), new ColorSensorIntaking(), new IdleShooter()),
                new SpinForTime(.5, 6).withInterrupt(RobotContainer.vision::hasTargets),
                new ParallelRaceGroup(
                                new TurnToAngleVision(true, false), new PrepareShooterForVision())
                        .withTimeout(3),
                new StopEverything());
    }
}
