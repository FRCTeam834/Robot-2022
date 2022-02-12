// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

// Imports
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase {
    /** Creates a new ExtendClimber. */
    public ExtendClimber() {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.climber);
    } /*

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {

          // Start moving the motors
          RobotContainer.climber.setRightMotor(Parameters.climber.DEFAULT_SPEED);
          RobotContainer.climber.setLeftMotor(Parameters.climber.DEFAULT_SPEED);
      }

      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {

          // Get the values from the encoders (makes logic faster, we don't need to poll every check)
          double rightPosition = RobotContainer.climber.getRightPosition();
          double leftPosition = RobotContainer.climber.getLeftPosition();

          // Make sure that the motors haven't passed their limits yet
          if (rightPosition >= Parameters.climber.MOVE_DISTANCE || leftPosition >= Parameters.climber.MOVE_DISTANCE) {

              // Check which motor needs stopped
              if (rightPosition >= Parameters.climber.MOVE_DISTANCE) {

                  // Stop the right motor, but keep the left motor running
                  RobotContainer.climber.setRightMotor(0);
                  RobotContainer.climber.setLeftMotor(Parameters.climber.DEFAULT_SPEED);
              }
              if (leftPosition >= Parameters.climber.MOVE_DISTANCE) {

                  // Stop the left motor, but keep the right motor running
                  RobotContainer.climber.setRightMotor(Parameters.climber.DEFAULT_SPEED);
                  RobotContainer.climber.setLeftMotor(0);
              }
          }
          // Check if one motor is ahead of the other by a significant amount
          else if (Math.abs(rightPosition - leftPosition) > Parameters.climber.ALLOWABLE_DEVIATION) {
              if (rightPosition > leftPosition) {
                  // Slow down the right motor, it's too far ahead
                  RobotContainer.climber.setRightMotor(Parameters.climber.DEFAULT_SPEED - Parameters.climber.SPEED_REDUCTION);
                  RobotContainer.climber.setLeftMotor(Parameters.climber.DEFAULT_SPEED + Parameters.climber.SPEED_REDUCTION);
              }
              else {
                  // Slow down the left motor, it's too far ahead
                  RobotContainer.climber.setRightMotor(Parameters.climber.DEFAULT_SPEED + Parameters.climber.SPEED_REDUCTION);
                  RobotContainer.climber.setLeftMotor(Parameters.climber.DEFAULT_SPEED - Parameters.climber.SPEED_REDUCTION);
              }
          }
          else {
              // There's nothing going on, we can just send the default speeds
              RobotContainer.climber.setRightMotor(Parameters.climber.DEFAULT_SPEED);
              RobotContainer.climber.setLeftMotor(Parameters.climber.DEFAULT_SPEED);
          }
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
          RobotContainer.climber.stopMotors();
      }

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
          return (RobotContainer.climber.getRightPosition() >= Parameters.climber.MOVE_DISTANCE &&
                  RobotContainer.climber.getLeftPosition() >= Parameters.climber.MOVE_DISTANCE);
      }*/
}
