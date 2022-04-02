// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import frc.robot.Parameters.climber.lift;
import frc.robot.Parameters.climber.tilt;

public class StupidClimbers extends SubsystemBase {

    // Tube objects
    public TelescopingTube rightTilt;
    public TelescopingTube leftTilt;
    public TelescopingTube rightLift;
    public TelescopingTube leftLift;

    /** Creates a new Climber. */
    public StupidClimbers() {

        rightTilt =
                new TelescopingTube(
                        "RT",
                        tilt.RIGHT_MOTOR_ID,
                        tilt.RIGHT_LIMIT_SWITCH_PORT,
                        tilt.SPOOL_CIRCUMFERENCE,
                        tilt.GEARBOX_RATIO,
                        tilt.kP,
                        tilt.kD,
                        tilt.MAX_DUTY,
                        tilt.CONTROL_TYPE,
                        tilt.HOME_DISTANCE,
                        tilt.RIGHT_LEGAL_DISTANCE,
                        tilt.POS_TOLERANCE,
                        true);
        leftTilt =
                new TelescopingTube(
                        "LT",
                        tilt.LEFT_MOTOR_ID,
                        tilt.LEFT_LIMIT_SWITCH_PORT,
                        tilt.SPOOL_CIRCUMFERENCE,
                        tilt.GEARBOX_RATIO,
                        tilt.kP,
                        tilt.kD,
                        tilt.MAX_DUTY,
                        tilt.CONTROL_TYPE,
                        tilt.HOME_DISTANCE,
                        tilt.LEFT_LEGAL_DISTANCE,
                        tilt.POS_TOLERANCE,
                        true);

        rightLift =
                new TelescopingTube(
                        "RL",
                        lift.RIGHT_MOTOR_ID,
                        lift.RIGHT_LIMIT_SWITCH_PORT,
                        lift.SPOOL_CIRCUMFERENCE,
                        lift.GEARBOX_RATIO,
                        lift.kP,
                        lift.kD,
                        lift.MAX_DUTY,
                        lift.CONTROL_TYPE,
                        lift.HOME_DISTANCE,
                        lift.UP_LEGAL_DISTANCE_RIGHT,
                        lift.POS_TOLERANCE,
                        true);
        leftLift =
                new TelescopingTube(
                        "LL",
                        lift.LEFT_MOTOR_ID,
                        lift.LEFT_LIMIT_SWITCH_PORT,
                        lift.SPOOL_CIRCUMFERENCE,
                        lift.GEARBOX_RATIO,
                        lift.kP,
                        lift.kD,
                        lift.MAX_DUTY,
                        lift.CONTROL_TYPE,
                        lift.HOME_DISTANCE,
                        lift.UP_LEGAL_DISTANCE_LEFT,
                        lift.POS_TOLERANCE,
                        true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Checks if all of the tubes are homed
     *
     * @return Are the tubes homed?
     */
    public boolean areTubesHomed() {
        // return (rightLift.isHomed()
        //         && leftLift.isHomed()
        //         && rightTilt.isHomed()
        //         && leftTilt.isHomed());

        return (leftTilt.isHomed());
    }

    /** Stops all of the tubes at once */
    public void stop() {
        // rightLift.stop();
        // leftLift.stop();
        // rightTilt.stop();
        leftTilt.stop();
    }

    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Climber");
            builder.addDoubleProperty("Left Lift Height", leftLift::getTubePosition, null);
            builder.addDoubleProperty("Left Tilt Height", leftTilt::getTubePosition, null);
            builder.addDoubleProperty("Right Lift Height", rightLift::getTubePosition, null);
            builder.addDoubleProperty("Rightt Height", rightTilt::getTubePosition, null);

        }
}
}
