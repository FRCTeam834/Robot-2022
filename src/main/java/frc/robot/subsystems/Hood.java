// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;

public class Hood extends SubsystemBase {
    /** Creates a new Hood. */

    // Motor objects
    CANSparkMax hoodMotor;

    RelativeEncoder hoodMotorEncoder;
    CachedPIDController pidController;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the hood has been homed yet
    boolean homed = false;

    public Hood() {
        // Initialize the hood motor
        hoodMotor = new CANSparkMax(Parameters.hood.MOTOR_ID, MotorType.kBrushless);
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.enableVoltageCompensation(12);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.setSmartCurrentLimit(10);
        hoodMotor.setInverted(false);

        // Set up the encoder of the hood motor
        hoodMotorEncoder = hoodMotor.getEncoder();
        hoodMotorEncoder.setPositionConversionFactor(360.0 / Parameters.hood.GEAR_RATIO);
        hoodMotorEncoder.setVelocityConversionFactor(360.0 / (Parameters.hood.GEAR_RATIO * 60));

        // Set up the PID controller
        pidController = new CachedPIDController(hoodMotor);

        // Set up the limit switch
        limitSwitch = new DigitalInput(Parameters.hood.LS_PORT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Make sure that motor doesn't overdrive the limit switch
        /*if (limitSwitch.get()) {
            hoodMotor.stopMotor();
            homed = true;
        }*/

        // Update the PID constants (if they are tunable)
        if (Parameters.tuningMode) {
            pidController.setP(Parameters.hood.pid.kP.get());
            pidController.setD(Parameters.hood.pid.kD.get());
        }
    }

    /**
     * Runs the motor at a set percentage
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
        pidController.setReference(deg, Parameters.hood.pid.CONTROL_TYPE);
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
        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) currentAngle);
        hoodMotor.setSoftLimit(
                SoftLimitDirection.kForward,
                (float) (currentAngle + Parameters.hood.ALLOWABLE_RANGE));

        // Enable the soft limits
        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the hood currently at home?
     */
    public boolean getLSValue() {
        return limitSwitch.get();
    }
}
