// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
            //start out by turning the robot to the proper angle and preparing the shooter based on distance
            //we never stop preparing the shooter until we force everything to end at the end, this is exactly what we want

            new ParallelCommandGroup(
                new TurnToAngleVision(), 
                new PrepareShooter()),

            //now run the indexer at full blast, but before you do that
            //wait until the shooter is ready to shoot (hood and shooter at their set point)
            //then, stop the indexer once the shot time is over
            new RunCommand(
                () -> RobotContainer.indexer.set(1)).beforeStarting(
                    new WaitUntilCommand(
                        RobotContainer.shooter::readyToShoot).withTimeout(
                            Parameters.shooter.SHOT_TIME)),

            //finally, do a parallel command group that tells everything to stop and to run the intake again, lets go score again
            new ParallelCommandGroup(new InstantCommand(RobotContainer.shooter::stop), new InstantCommand(RobotContainer.indexer::stop), new InstantCommand(() -> RobotContainer.indexer.set(Parameters.intake.INTAKE_SPEED))));

                

    }
}
