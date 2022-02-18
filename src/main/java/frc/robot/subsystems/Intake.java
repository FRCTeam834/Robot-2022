// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
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

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    // Motor object for spool
    CANSparkMax spoolMotor;

    // Motor encoder
    RelativeEncoder spoolMotorEncoder;

    // spool PID controller
    CachedPIDController pidController;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the spool has been homed yet
    boolean homed = false;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Invert the direction
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(Parameters.intake.INTAKE_MOTOR_CURRENT_LIMIT);

        // Spool stuff
        // Initialize the spool motor
        spoolMotor = new CANSparkMax(Parameters.intake.spool.MOTOR_ID, MotorType.kBrushless);
        spoolMotor.restoreFactoryDefaults();
        spoolMotor.enableVoltageCompensation(12);
        spoolMotor.setIdleMode(IdleMode.kBrake);
        spoolMotor.setSmartCurrentLimit(10);
        spoolMotor.setInverted(true);

        // Set up the encoder of the spool motor
        spoolMotorEncoder = spoolMotor.getEncoder();
        spoolMotorEncoder.setPositionConversionFactor(
                Parameters.intake.spool.CIRCUMFRENCE / Parameters.intake.spool.GEARBOX_RATIO);
        spoolMotorEncoder.setVelocityConversionFactor(
                Parameters.intake.spool.CIRCUMFRENCE
                        / (Parameters.intake.spool.GEARBOX_RATIO * 60));

        // Set up the PID controller
        pidController = new CachedPIDController(spoolMotor);
        pidController.setOutputRange(
                -Parameters.intake.spool.MAX_MOTOR_DUTY, Parameters.intake.spool.MAX_MOTOR_DUTY);
        pidController.setP(Parameters.intake.spool.pid.kP.get());
        pidController.setD(Parameters.intake.spool.pid.kD.get());

        // Set up the limit switch
        limitSwitch = new DigitalInput(Parameters.intake.spool.LS_PORT);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the PID constants (if they are tunable)
        if (Parameters.tuningMode) {
            pidController.setP(Parameters.intake.spool.pid.kP.get());
            pidController.setD(Parameters.intake.spool.pid.kD.get());
        }
    }

    // Turns on the intake
    public void turnOn() {
        intakeMotor.set(Parameters.intake.INTAKE_SPEED);
    }

    public void runSpoolMotor(double percent) {
        spoolMotor.set(percent);
    }

    public void stopSpoolMotor() {
        spoolMotor.stopMotor();
    }

    // Makes the intake go in reverse
    public void spitItOut() {
        intakeMotor.set(-Parameters.intake.INTAKE_SPEED);
    }

    // Stops the intake motors
    public void stop() {
        intakeMotor.set(0);
    }

    /**
     * Tells the spool to move to a specific angle
     *
     * @param dist The angle, in degrees, to move the spool to
     */
    public void setDesiredDistance(double dist) {

        // Set the motor's distance if homed
        if (homed) {
            pidController.setReference(dist, Parameters.intake.spool.pid.CONTROL_TYPE);
        }
        else {
            System.out.println("SPOOL NOT HOMED!!!");
        }

        // Print out the angle information if desired
        if (Parameters.debug) {
            System.out.println(
                    String.format("S: %.2f | A: %.2f", dist, spoolMotorEncoder.getPosition()));
        }
    }

    /** Returns the desired distance of the spool */
    public double getDesiredDistance() {

        // Set the motor's distance if homed
        if (homed) {
            return pidController.getReference();
        } else {
            System.out.println("Distancing not available till homed!");
            return 0;
        }
    }

    /**
     * Sets the current angle of the spool. This should be used when homing the spool.
     *
     * @param currentDistance
     */
    public void setCurrentDistance(double currentDistance) {

        // Set the current position
        spoolMotorEncoder.setPosition(currentDistance);

        // Set the soft limits
        // Soft limits are basically the controller not allowing certain values to be set for the
        // PID loop
        spoolMotor.setSoftLimit(
                SoftLimitDirection.kForward, (float) Parameters.intake.spool.DOWN_DISTANCE);
        spoolMotor.setSoftLimit(
                SoftLimitDirection.kReverse, (float) (Parameters.intake.spool.UP_DISTANCE));

        // Enable the soft limits
        spoolMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        spoolMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        // Set that the spool is homed
        homed = true;
    }

    /**
     * Returns the position of the spool motor
     *
     * @return The position, in m
     */
    public double getSpoolPosition() {
        return spoolMotorEncoder.getPosition();
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the spool currently at home?
     */
    public boolean getLSValue() {
        return limitSwitch.get();
    }
}
