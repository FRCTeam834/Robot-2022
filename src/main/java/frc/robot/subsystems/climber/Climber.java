// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

public class Climber extends SubsystemBase {

    // Tube objects
    public TelescopingTube rightTilt;
    public TelescopingTube leftTilt;
    public TelescopingTube rightLift;
    public TelescopingTube leftLift;

    /** Creates a new Climber. */
    public Climber() {

        // Initialize the tubes
        rightTilt = new TelescopingTube(
            "RT", 
            Parameters.climber.tilt.RIGHT_MOTOR_ID, 
            Parameters.climber.tilt.RIGHT_LIMIT_SWITCH_PORT, 
            Parameters.climber.tilt.SPOOL_CIRCUMFERENCE, 
            Parameters.climber.tilt.GEARBOX_RATIO, 
            Parameters.climber.tilt.kP, 
            Parameters.climber.tilt.kD, 
            Parameters.climber.tilt.MAX_DUTY, 
            Parameters.climber.tilt.CONTROL_TYPE, 
            Parameters.climber.tilt.HOME_DISTANCE, 
            Parameters.climber.tilt.UP_DISTANCE);
        leftTilt = new TelescopingTube(
            "LT", 
            Parameters.climber.tilt.LEFT_MOTOR_ID, 
            Parameters.climber.tilt.LEFT_LIMIT_SWITCH_PORT, 
            Parameters.climber.tilt.SPOOL_CIRCUMFERENCE, 
            Parameters.climber.tilt.GEARBOX_RATIO, 
            Parameters.climber.tilt.kP, 
            Parameters.climber.tilt.kD, 
            Parameters.climber.tilt.MAX_DUTY, 
            Parameters.climber.tilt.CONTROL_TYPE, 
            Parameters.climber.tilt.HOME_DISTANCE, 
            Parameters.climber.tilt.UP_DISTANCE);
            
        rightLift = new TelescopingTube(
            "RL", 
            Parameters.climber.lift.RIGHT_MOTOR_ID, 
            Parameters.climber.lift.RIGHT_LIMIT_SWITCH_PORT, 
            Parameters.climber.lift.SPOOL_CIRCUMFERENCE, 
            Parameters.climber.lift.GEARBOX_RATIO, 
            Parameters.climber.lift.kP, 
            Parameters.climber.lift.kD, 
            Parameters.climber.lift.MAX_DUTY, 
            Parameters.climber.lift.CONTROL_TYPE, 
            Parameters.climber.lift.HOME_DISTANCE, 
            Parameters.climber.lift.UP_DISTANCE);
        leftLift = new TelescopingTube(
            "LL", 
            Parameters.climber.lift.LEFT_MOTOR_ID, 
            Parameters.climber.lift.LEFT_LIMIT_SWITCH_PORT, 
            Parameters.climber.lift.SPOOL_CIRCUMFERENCE, 
            Parameters.climber.lift.GEARBOX_RATIO, 
            Parameters.climber.lift.kP, 
            Parameters.climber.lift.kD, 
            Parameters.climber.lift.MAX_DUTY, 
            Parameters.climber.lift.CONTROL_TYPE, 
            Parameters.climber.lift.HOME_DISTANCE, 
            Parameters.climber.lift.UP_DISTANCE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        rightLift.stop();
        leftLift.stop();
        rightTilt.stop();
        leftTilt.stop();
    }
}
