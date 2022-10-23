// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.driving.DriveUntilAngle;
import frc.robot.subsystems.climber.HomeClimberTubes;
import frc.robot.utilityClasses.LEDColors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
    /** Creates a new Climb. */

    public Climb() {

        // Set the LEDs to red, we're climbing baby!
        RobotContainer.leds.setPrimaryColor(LEDColors.STROBE_RED);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

                // Stop all of the unused motors (includes the drivetrain)
                new InstantCommand(() -> RobotContainer.intake.stop()),
                new InstantCommand(() -> RobotContainer.indexer.stop()),
                new InstantCommand(() -> RobotContainer.shooter.stop()),
                new InstantCommand(() -> RobotContainer.hood.stop()),
                new InstantCommand(() -> RobotContainer.intakeWinch.stop()),
                new InstantCommand(() -> RobotContainer.driveTrain.stop()),

                // Home the climber tubes
                new HomeClimberTubes(),
                new PrintCommand("Homed!"),
                // Now extend the tubes

                new ParallelCommandGroup(
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftLift,
                                (Parameters.climber.lift.UP_LEGAL_DISTANCE_LEFT),
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightLift,
                                (Parameters.climber.lift.UP_LEGAL_DISTANCE_RIGHT),
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftTilt,
                                (Parameters.climber.tilt.LEFT_LEGAL_DISTANCE),
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightTilt,
                                (Parameters.climber.tilt.RIGHT_LEGAL_DISTANCE),
                                1)),

                // Tilt the robot
                new InstantCommand(() -> RobotContainer.driveTrain.setDesiredAngles(0, 0, 0, 0)),
                new WaitCommand(0.25),

                new InstantCommand(() -> RobotContainer.canManualLift = true),

                new DriveUntilAngle(
                                Parameters.climber.DRIVE_TILT_SPEED,
                                Parameters.climber.ROBOT_TILT_ANGLE)
                        .withTimeout(5),

                new InstantCommand(() -> RobotContainer.canManualLift = false),

                // Grab the 3rd rung bar
                new ParallelCommandGroup(
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftLift,
                                Parameters.climber.lift.GRAB_DISTANCE,
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightLift,
                                Parameters.climber.lift.GRAB_DISTANCE,
                                1)),

                // We lifting boys!
                new ParallelCommandGroup(
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftTilt,
                                Parameters.climber.tilt.LEFT_HALF_DISTANCE,
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightTilt,
                                Parameters.climber.tilt.RIGHT_HALF_DISTANCE,
                                1)),
                new ParallelCommandGroup(
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftLift,
                                Parameters.climber.lift.DOWN_DISTANCE,
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightLift,
                                Parameters.climber.lift.DOWN_DISTANCE,
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.leftTilt,
                                Parameters.climber.tilt.DOWN_DISTANCE,
                                1),
                        new MoveTubeToPosition(
                                RobotContainer.climbers2.rightTilt,
                                Parameters.climber.tilt.DOWN_DISTANCE,
                                1)),

                // ! The climb is done... PARTY TIME!!!
                new InstantCommand(() -> RobotContainer.leds.setPrimaryColor(LEDColors.PARTY)));
    }
}
