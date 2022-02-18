// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;

public class Climber extends SubsystemBase {

    // Motor objects
    private CANSparkMax rightSpoolMotor;
    private CANSparkMax leftSpoolMotor;

    private CANSparkMax rightTiltMotor;
    private CANSparkMax leftTiltMotor;

    // Encoder objects (from NEOs)
    private RelativeEncoder rightSpoolEncoder;
    private RelativeEncoder leftSpoolEncoder;
    private RelativeEncoder rightTiltEncoder;
    private RelativeEncoder leftTiltEncoder;

    // Limit Switches
    DigitalInput rightLiftLimitSwitch;
    DigitalInput leftLiftLimitSwitch;
    DigitalInput rightTiltLimitSwitch;
    DigitalInput leftTiltLimitSwitch;

    // PID controllers
    CachedPIDController rightLiftPidController;
    CachedPIDController leftLiftPidController;
    CachedPIDController rightTiltPidController;
    CachedPIDController leftTiltPidController;

    /** Creates a new Climber. */
    public Climber() {

        // Initialize the right spool motor
        rightSpoolMotor =
                new CANSparkMax(Parameters.climber.lift.RIGHT_SPOOL_MOTOR_ID, MotorType.kBrushless);
        rightSpoolMotor.restoreFactoryDefaults();
        rightSpoolMotor.enableVoltageCompensation(12);
        rightSpoolMotor.setIdleMode(IdleMode.kBrake);
        rightSpoolMotor.setSmartCurrentLimit(10);
        rightSpoolMotor.setInverted(true);

        // Initialize the left spool motor
        leftSpoolMotor =
                new CANSparkMax(Parameters.climber.lift.LEFT_SPOOL_MOTOR_ID, MotorType.kBrushless);
        leftSpoolMotor.restoreFactoryDefaults();
        leftSpoolMotor.enableVoltageCompensation(12);
        leftSpoolMotor.setIdleMode(IdleMode.kBrake);
        leftSpoolMotor.setSmartCurrentLimit(10);
        leftSpoolMotor.setInverted(true);

        // Initialize the right tilt motor
        rightTiltMotor =
                new CANSparkMax(Parameters.climber.tilt.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);
        rightTiltMotor.restoreFactoryDefaults();
        rightTiltMotor.enableVoltageCompensation(12);
        rightTiltMotor.setIdleMode(IdleMode.kBrake);
        rightTiltMotor.setSmartCurrentLimit(10);
        rightTiltMotor.setInverted(true);

        // Initialize the left tilt motor
        leftTiltMotor =
                new CANSparkMax(Parameters.climber.tilt.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);
        leftTiltMotor.restoreFactoryDefaults();
        leftTiltMotor.enableVoltageCompensation(12);
        leftTiltMotor.setIdleMode(IdleMode.kBrake);
        leftTiltMotor.setSmartCurrentLimit(10);
        leftTiltMotor.setInverted(true);

        rightLiftLimitSwitch = new DigitalInput(Parameters.climber.lift.RIGHT_LIMIT_SWITCH_PORT);
        leftLiftLimitSwitch = new DigitalInput(Parameters.climber.lift.LEFT_LIMIT_SWITCH_PORT);
        rightTiltLimitSwitch = new DigitalInput(Parameters.climber.tilt.RIGHT_LIMIT_SWITCH_PORT);
        leftTiltLimitSwitch = new DigitalInput(Parameters.climber.tilt.LEFT_LIMIT_SWITCH_PORT);

        CANSparkMax rightMotor = new CANSparkMax(Parameters.climber.RIGHT_ID, MotorType.kBrushless);

        // Get the encoders from the motors
        rightSpoolEncoder = rightSpoolMotor.getEncoder();
        leftSpoolEncoder = leftSpoolMotor.getEncoder();
        rightTiltEncoder = rightTiltMotor.getEncoder();
        leftTiltEncoder = leftTiltMotor.getEncoder();

        // Set up the encoder of the right spool motor
        rightSpoolEncoder.setPositionConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE
                        / Parameters.climber.lift.SPOOL_GEARBOX_RATIO);
        rightSpoolEncoder.setVelocityConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE
                        / (Parameters.climber.lift.SPOOL_GEARBOX_RATIO * 60)); // divide once

        // Set up the encoder of the left spool motor
        leftSpoolEncoder.setPositionConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE
                        / Parameters.climber.lift.SPOOL_GEARBOX_RATIO);
        leftSpoolEncoder.setVelocityConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE
                        / (Parameters.climber.lift.SPOOL_GEARBOX_RATIO * 60)); // divide once

        // Set the position conversion factors for the tilt motors
        rightTiltEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);
        leftTiltEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);

        // Set the current position of the climber to 0
        rightSpoolEncoder.setPosition(0);
        leftSpoolEncoder.setPosition(0);
        rightTiltEncoder.setPosition(0);
        leftTiltEncoder.setPosition(0);

        // Set up the PID controller
        rightLiftPidController = new CachedPIDController(rightSpoolMotor);
        rightLiftPidController.setOutputRange(
                -Parameters.climber.lift.RIGHT_MAX_MOTOR_DUTY,
                Parameters.climber.lift.RIGHT_MAX_MOTOR_DUTY);
        rightLiftPidController.setP(Parameters.intake.spool.pid.kP.get());
        rightLiftPidController.setD(Parameters.intake.spool.pid.kD.get());

        leftLiftPidController = new CachedPIDController(leftSpoolMotor);
        leftLiftPidController.setOutputRange(
                -Parameters.climber.lift.LEFT_MAX_MOTOR_DUTY,
                Parameters.climber.lift.LEFT_MAX_MOTOR_DUTY);
        leftLiftPidController.setP(Parameters.intake.spool.pid.kP.get());
        leftLiftPidController.setD(Parameters.intake.spool.pid.kD.get());

        rightTiltPidController = new CachedPIDController(rightTiltMotor);
        rightTiltPidController.setOutputRange(
                -Parameters.climber.tilt.RIGHT_MAX_MOTOR_DUTY,
                Parameters.climber.tilt.RIGHT_MAX_MOTOR_DUTY);
        rightTiltPidController.setP(Parameters.intake.spool.pid.kP.get());
        rightTiltPidController.setD(Parameters.intake.spool.pid.kD.get());

        leftTiltPidController = new CachedPIDController(leftTiltMotor);
        leftTiltPidController.setOutputRange(
                -Parameters.climber.tilt.LEFT_MAX_MOTOR_DUTY,
                Parameters.climber.tilt.LEFT_MAX_MOTOR_DUTY);
        leftTiltPidController.setP(Parameters.intake.spool.pid.kP.get());
        leftTiltPidController.setD(Parameters.intake.spool.pid.kD.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getRightSpoolPosition() {
        return rightSpoolEncoder.getPosition();
    }

    public double getLeftSpoolPosition() {
        return leftSpoolEncoder.getPosition();
    }

    public double getRightTiltPosition() {
        return rightTiltEncoder.getPosition();
    }

    public double getLeftTiltPosition() {
        return leftTiltEncoder.getPosition();
    }

    /*public double getLeftPosition() {
       return leftEncoder.getPosition();
    }*/

    /*
    public void setRightMotor(double speed) {
        rightMotor.set(speed);
    }

    public void runRightMotor() {
        rightMotor.set(Parameters.climber.DEFAULT_SPEED);
    }

    public void runRightMotorBackward() {
        rightMotor.set(-Parameters.climber.DEFAULT_SPEED);
    }
    */

    /*public void setLeftMotor(double speed) {
        leftMotor.set(speed);
    }*/

    public void stopMotors() {
        rightSpoolMotor.stopMotor();
        leftSpoolMotor.stopMotor();
        rightTiltMotor.stopMotor();
        leftTiltMotor.stopMotor();
    }
}
