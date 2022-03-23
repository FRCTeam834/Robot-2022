// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.StopEverything;
import frc.robot.commands.indexing.IndexForTime;
import frc.robot.commands.swerve.TurnToAngleVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
    /** Creates a new AutoShoot. */
    public AutoShoot() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(
                                new TurnToAngleVision(false, true), new PrepareShooterForVision())
                        .withInterrupt(RobotContainer.shooter::readyToShoot),
                new IndexForTime(3),
                new StopEverything());

        // start out by turning the robot to the proper angle and preparing the shooter
        // based on distance
        // we never stop preparing the shooter until we force everything to end at the end,
        // this is exactly what we want

        // now run the indexer at full blast, but before you do that
        // wait until the shooter is ready to shoot (hood and shooter at their set point)
        // then, stop the indexer once the shot time is over

    }
}
