// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.utilityClasses.LEDColors;

public class GrabAndLift extends SequentialCommandGroup {
    public GrabAndLift() {
        addCommands(
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
