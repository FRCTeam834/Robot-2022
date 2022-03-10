// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.HomeTube;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HomeClimberTubes extends ParallelCommandGroup {
    /** Creates a new HomeClimberTubes. */
    public HomeClimberTubes() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            
                 new HomeTube(
                         RobotContainer.climbers2.rightLift,
                         Parameters.climber.lift.HOME_SPEED,
                         Parameters.climber.lift.HOME_DISTANCE),
                 new HomeTube(
                         RobotContainer.climbers2.leftLift,
                         Parameters.climber.lift.HOME_SPEED,
                         Parameters.climber.lift.HOME_DISTANCE),
                         
                 new HomeTube(
                         RobotContainer.climbers2.rightTilt,
                         Parameters.climber.tilt.HOME_SPEED,
                         Parameters.climber.tilt.HOME_DISTANCE),
                         
                new HomeTube(
                        RobotContainer.climbers2.leftTilt,
                        Parameters.climber.tilt.HOME_SPEED,
                        Parameters.climber.tilt.HOME_DISTANCE));
                        
    }
}
