// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
    /** Creates a new Climb. */
    public Climb() {
        /*
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(

                // Each of the following lines will run one after the other:

                // First, move both the tilt and lift tubes up
                new ParallelCommandGroup(),
                       // new MoveTubeToPosition(
                         //       RobotContainer.climber.leftTilt,
                           //     Parameters.climber.tilt.UP_DISTANCE)),
                // new MoveTubeToPosition(
                //         RobotContainer.climber.rightTilt,
                //         Parameters.climber.tilt.UP_DISTANCE),
                // new MoveTubeToPosition(
                //         RobotContainer.climber.leftLift,
                //         Parameters.climber.lift.UP_DISTANCE),
                // new MoveTubeToPosition(
                //         RobotContainer.climber.rightLift,
                //         Parameters.climber.lift.UP_DISTANCE)),

                // Then... move the robot until we tip to the desired angle
                new DriveUntilAngle(
                        Parameters.climber.DRIVE_TILT_SPEED, Parameters.climber.ROBOT_TILT_ANGLE),

                // Engage the lifting hooks (will prevent the robot from falling when tilt tubes are
                // retracted)
                // new ParallelCommandGroup(
                //         new MoveTubeToPosition(
                //                 RobotContainer.climber.leftLift,
                //                 Parameters.climber.lift.GRAB_DISTANCE),
                //         new MoveTubeToPosition(
                //                 RobotContainer.climber.rightLift,
                //                 Parameters.climber.lift.GRAB_DISTANCE)),

                // Lower the tilt tubes
               // new ParallelCommandGroup(
                 //       new MoveTubeToPosition(
                   //             RobotContainer.climber.leftTilt,
                     //           Parameters.climber.tilt.DOWN_DISTANCE)));
        //         new MoveTubeToPosition(
        //                 RobotContainer.climber.rightTilt,
        //                 Parameters.climber.tilt.DOWN_DISTANCE)),

        // // Lift the robot off the ground
        // // TO THE MOON BABY!!!!
        // new ParallelCommandGroup(
        //         new MoveTubeToPosition(
        //                 RobotContainer.climber.leftLift,
        //                 Parameters.climber.lift.LIFT_DISTANCE),
        //         new MoveTubeToPosition(
        //                 RobotContainer.climber.rightLift,
        //                 Parameters.climber.lift.LIFT_DISTANCE)));
        */
    }
}
