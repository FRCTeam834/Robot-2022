// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Parameters;
import frc.robot.Robot;
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
        new InstantCommand(() -> RobotContainer.indexer.set(-.15)).withTimeout(.5),
        new RunCommand(() -> RobotContainer.shooter.setDesiredSpeed(Parameters.shooter.SHOT_SPEED), RobotContainer.shooter),
        new WaitUntilCommand(RobotContainer.shooter::isAtSetPoint).andThen(() -> RobotContainer.indexer.set(1)).andThen(new WaitCommand(Parameters.shooter.SHOT_TIME)),
        new ParallelCommandGroup(new InstantCommand(RobotContainer.shooter::stop), new InstantCommand(RobotContainer.indexer::stop))
    );
  }
}
