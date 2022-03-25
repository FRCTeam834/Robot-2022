// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.Parameters.intake.spool;
import frc.robot.utilityClasses.CachedPIDController;

public class IntakeWinch extends SubsystemBase {
    /** Creates a new IntakeWinch. */
    // Motor object for spool
    CANSparkMax spoolMotor;
    Debouncer debouncer = new Debouncer(.2);

    // Motor encoder
    RelativeEncoder spoolMotorEncoder;

    // spool PID controller
    CachedPIDController pidController;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the spool has been homed yet
    boolean homed = false;

    public IntakeWinch() {
        // Initialize the spool motor
        spoolMotor = new CANSparkMax(Parameters.intake.spool.MOTOR_ID, MotorType.kBrushless);
        spoolMotor.restoreFactoryDefaults();
        spoolMotor.enableVoltageCompensation(12);
        spoolMotor.setIdleMode(IdleMode.kBrake);
        spoolMotor.setSmartCurrentLimit(Parameters.intake.spool.MOTOR_CURRENT_LIMIT);
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
        if (Parameters.tuningMode) {
            pidController.setP(Parameters.intake.spool.pid.kP.get());
            pidController.setD(Parameters.intake.spool.pid.kD.get());
        }
    }

    public void set(double percent) {
        spoolMotor.set(percent);
    }

    public void stop() {
        spoolMotor.stopMotor();
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
        } else {
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
        //spoolMotor.setSoftLimit(
        //        SoftLimitDirection.kForward, (float) Parameters.intake.spool.DOWN_DISTANCE);
        //spoolMotor.setSoftLimit(
        //        SoftLimitDirection.kReverse, (float) (Parameters.intake.spool.UP_DISTANCE));

        // Enable the soft limits
        //spoolMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //spoolMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

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
     * Checks if the intake is homed yet
     *
     * @return Is the intake homed yet?
     */
    public boolean isHomed() {
        return homed;
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the spool currently at home?
     */
    public boolean getLSValue() {
        return limitSwitch.get();
    }

    // sets current limit of the spool motor
    public void setCurrentLimit(int limit) {
        spoolMotor.setSmartCurrentLimit(limit);
    }

    // Gets the current of the motor (in A)
    public double getMotorCurrent() {
        return spoolMotor.getOutputCurrent();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("IntakeWinch");
            builder.addDoubleProperty("Current", this::getMotorCurrent, null);
            builder.addDoubleProperty("Position", this::getSpoolPosition, null);
            builder.addDoubleProperty("Desired", this::getDesiredDistance, null);
        }
    }
}
