// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.driving.DriveUntilAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
    /** Creates a new Climb. */
    public Climb() {

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            // TODO: TEST THIS
            //new InstantCommand(() -> RobotContainer.navX.resetPitch()),
            new ParallelCommandGroup(
            new MoveTubeToPosition(RobotContainer.climbers2.leftLift, Parameters.climber.lift.UP_LEGAL_DISTANCE, .25),
            new MoveTubeToPosition(RobotContainer.climbers2.rightLift, Parameters.climber.lift.UP_LEGAL_DISTANCE, .25),
            new MoveTubeToPosition(RobotContainer.climbers2.leftTilt, Parameters.climber.tilt.LEFT_LEGAL_DISTANCE, .25),
            new MoveTubeToPosition(RobotContainer.climbers2.rightTilt, Parameters.climber.tilt.RIGHT_LEGAL_DISTANCE, .25)),

            new DriveUntilAngle(-1, Parameters.climber.ROBOT_TILT_ANGLE).withTimeout(5),

            new ParallelCommandGroup(
                new MoveTubeToPosition(RobotContainer.climbers2.leftLift, Parameters.climber.lift.GRAB_DISTANCE, 1),
                new MoveTubeToPosition(RobotContainer.climbers2.rightLift, Parameters.climber.lift.GRAB_DISTANCE, 1)),
            new ParallelCommandGroup(
                new MoveTubeToPosition(RobotContainer.climbers2.leftTilt, Parameters.climber.tilt.DOWN_DISTANCE, 1),
                new MoveTubeToPosition(RobotContainer.climbers2.rightTilt, Parameters.climber.tilt.DOWN_DISTANCE, 1)),
            new ParallelCommandGroup(
                new MoveTubeToPosition(RobotContainer.climbers2.leftLift, Parameters.climber.lift.DOWN_DISTANCE, 1),
                new MoveTubeToPosition(RobotContainer.climbers2.rightLift, Parameters.climber.lift.DOWN_DISTANCE, 1)));
    }
}
