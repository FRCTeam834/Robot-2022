// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    // The encoder motor object
    RelativeEncoder intakeMotorEncoder;

    // The PID controller for the intake
    PIDController intakeMotorPID;

    // The feed forward of the motor (helps to correct for friction and other forces)
    SimpleMotorFeedforward intakeMotorFF = new SimpleMotorFeedforward(0.12608, 0.344, 0.032386);

    // Store if we're using PID
    boolean usingPID = false;

    // Store the set velocity
    double setVelocity = 0;

    // Homing limit switch
    DigitalInput limitSwitch;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Configure the motor's settings
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);

        // Set the current limit
        intakeMotor.setSmartCurrentLimit(Parameters.intake.INTAKE_MOTOR_CURRENT_LIMIT);

        // Reduce the CAN bus usage by reducing the update frequency of position frame
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);

        // Get the encoder of the shooter motor
        intakeMotorEncoder = intakeMotor.getEncoder();

        // Set up the encoder's conversion factor
        // Multiply RPM by the circumference and 60 seconds to get m/s
        intakeMotorEncoder.setVelocityConversionFactor(
                (Parameters.intake.FRONT_WHEEL_DIA_M * Math.PI) / 60);

        // Save all of the configured settings
        intakeMotor.burnFlash();

        // Set up the PID controller
        intakeMotorPID = new PIDController(0.024089, 0, 0);
    }

    public void set(double percentage) {
        usingPID = false;
        intakeMotor.set(percentage);
    }

    public void setDesiredSpeed(double setpoint) {
        setVelocity = setpoint;
        usingPID = true;
    }

    public double getSetpoint() {
        return setVelocity;
    }

    public void stop() {
        usingPID = false;
        intakeMotor.stopMotor();
    }

    public void periodic() {
        if (usingPID) {
            intakeMotor.setVoltage(
                    intakeMotorPID.calculate(intakeMotorEncoder.getVelocity(), setVelocity) * 12
                            + .9 * intakeMotorFF.calculate(setVelocity));
        }
    }

    public double getSpeed() {
        return intakeMotorEncoder.getVelocity();
    }
}
