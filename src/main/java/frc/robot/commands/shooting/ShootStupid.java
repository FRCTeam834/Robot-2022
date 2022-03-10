// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootStupid extends SequentialCommandGroup {
    /** Creates a new ShootStupid. */
    public ShootStupid() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new PrintCommand("Stage 1"),
                new InstantCommand(() -> RobotContainer.indexer.set(-.15)).withTimeout(.5).andThen(() -> RobotContainer.indexer.set(0)),
                new PrintCommand("Stage 2"),
                new InstantCommand(
                        () -> RobotContainer.shooter.set(.25),
                        RobotContainer.shooter),
                new PrintCommand("Stage 3"),
                new WaitCommand
                (2)
                        .andThen(() -> RobotContainer.indexer.set(.15))
                        .andThen(new WaitCommand(1)),
                new ParallelCommandGroup(
                        new InstantCommand(RobotContainer.shooter::stop),
                        new InstantCommand(RobotContainer.indexer::stop)));
    }
}
