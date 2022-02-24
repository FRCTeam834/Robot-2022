// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;

public class Hood extends SubsystemBase {

    // Motor object
    CANSparkMax hoodMotor;

    // Motor encoder
    RelativeEncoder hoodMotorEncoder;

    // Hood PID controller
    CachedPIDController pidController;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the hood has been homed yet
    boolean homed = false;

    /** Creates a new Hood. */
    public Hood() {
        // Initialize the hood motor
        hoodMotor = new CANSparkMax(Parameters.hood.MOTOR_ID, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.enableVoltageCompensation(12);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.setSmartCurrentLimit(10);
        hoodMotor.setInverted(true);
        hoodMotor.setSmartCurrentLimit(Parameters.hood.CURRENT_LIMIT);

        // Set up the encoder of the hood motor
        hoodMotorEncoder = hoodMotor.getEncoder();
        hoodMotorEncoder.setPositionConversionFactor(
                360.0 / (Parameters.hood.GEARBOX_RATIO * Parameters.hood.CHAIN_RATIO));
        hoodMotorEncoder.setVelocityConversionFactor(
                360.0 / (Parameters.hood.GEARBOX_RATIO * Parameters.hood.CHAIN_RATIO * 60));

        // Set up the PID controller
        pidController = new CachedPIDController(hoodMotor);
        pidController.setOutputRange(
                -Parameters.hood.MAX_MOTOR_DUTY, Parameters.hood.MAX_MOTOR_DUTY);

        // Set up the limit switch
        limitSwitch = new DigitalInput(Parameters.hood.LS_PORT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update the PID constants (if they are tunable)
        if (Parameters.tuningMode) {
            pidController.setP(Parameters.hood.pid.kP.get());
            pidController.setD(Parameters.hood.pid.kD.get());
        }
    }

    /**
     * Runs the motor at a set percentage !NOTE: IGNORES LIMITS, WILL RESULT IN OVERDRIVING
     *
     * @param percent to run motor at
     */
    public void runMotor(double percent) {
        hoodMotor.set(percent);
    }

    /** Stops the hood */
    public void stop() {
        hoodMotor.stopMotor();
    }

    /**
     * Tells the hood to move to a specific angle
     *
     * @param deg The angle, in degrees, to move the hood to
     */
    public void setDesiredAngle(double deg) {

        // Set the motor's angle if homed
        if (homed) {
            pidController.setReference(deg, Parameters.hood.pid.CONTROL_TYPE);
        }

        // Print out the angle information if desired
        if (Parameters.debug) {
            System.out.println(
                    String.format("S: %.2f | A: %.2f", deg, hoodMotorEncoder.getPosition()));
        }
    }

    /**
     * Returns the desired angle of the hood
     *
     * @return The desired angle (deg)
     */
    public double getDesiredAngle() {
        return pidController.getReference();
    }

    /**
     * Sets the current angle of the hood. This should be used when homing the hood.
     *
     * @param currentAngle
     */
    public void setCurrentAngle(double currentAngle) {

        // Set the current position
        hoodMotorEncoder.setPosition(currentAngle);

        // Set the soft limits
        // Soft limits are basically the controller not allowing certain values to be set for the
        // PID loop
        hoodMotor.setSoftLimit(SoftLimitDirection.kForward, (float) currentAngle);
        hoodMotor.setSoftLimit(
                SoftLimitDirection.kReverse,
                (float) (currentAngle - Parameters.hood.ALLOWABLE_RANGE));

        // Enable the soft limits
        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        // Set that the hood is homed
        homed = true;
    }

    // Gets the current angle of the hood
    public double getCurrentAngle() {
        return hoodMotorEncoder.getPosition();
    }

    /**
     * Checks if the hood is at the setpoint
     *
     * @return Is the hood at the desired angle?
     */
    public boolean isAtDesiredAngle() {
        return (Math.abs(getCurrentAngle() - getDesiredAngle()) < Parameters.hood.ANGLE_TOLERANCE);
    }

    /**
     * Checks if the hood is homed yet
     *
     * @return Is the hood homed yet?
     */
    public boolean isHomed() {
        return homed;
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the hood currently at home?
     */
    public boolean getLSValue() {
        return limitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Shooter");
            builder.addDoubleProperty(
                    "Angle", hoodMotorEncoder::getPosition, this::setDesiredAngle);
            builder.addBooleanProperty("Limit Switch", this::getLSValue, null);
        }
    }
}
