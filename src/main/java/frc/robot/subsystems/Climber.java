// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Parameters;

public class Climber extends SubsystemBase {

    // Motor objects

    private CANSparkMax rightSpoolMotor;
    private CANSparkMax leftSpoolMotor;

    private CANSparkMax rightPivotMotor;
    private CANSparkMax leftPivotMotor;

    // Encoder objects (from NEOs)
    private RelativeEncoder rightSpoolEncoder;
    private RelativeEncoder leftSpoolEncoder;
    private RelativeEncoder rightPivotEncoder;
    private RelativeEncoder leftPivotEncoder;



    // Limit Switch
    DigitalInput rightLimitSwitch;
    DigitalInput leftLimitSwitch;

    // DigitalInput rightBottomLimitSwitch;
    // DigitalInput leftBottomLimitSwitch;

    /** Creates a new Climber. */
    public Climber() {
        // Create the motors

        // Initialize the right spool motor
        rightSpoolMotor = new CANSparkMax(Parameters.climber.right.SPOOL_MOTOR_ID, MotorType.kBrushless);
        rightSpoolMotor.restoreFactoryDefaults();
        rightSpoolMotor.enableVoltageCompensation(12);
        rightSpoolMotor.setIdleMode(IdleMode.kBrake);
        rightSpoolMotor.setSmartCurrentLimit(10);
        rightSpoolMotor.setInverted(true);

        // Initialize the left spool motor
        leftSpoolMotor = new CANSparkMax(Parameters.climber.left.SPOOL_MOTOR_ID, MotorType.kBrushless);
        leftSpoolMotor.restoreFactoryDefaults();
        leftSpoolMotor.enableVoltageCompensation(12);
        leftSpoolMotor.setIdleMode(IdleMode.kBrake);
        leftSpoolMotor.setSmartCurrentLimit(10);
        leftSpoolMotor.setInverted(true);

        // Initialize the right pivot motor
        rightPivotMotor = new CANSparkMax(Parameters.climber.right.PIVOT_MOTOR_ID, MotorType.kBrushless);
        rightPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.enableVoltageCompensation(12);
        rightPivotMotor.setIdleMode(IdleMode.kBrake);
        rightPivotMotor.setSmartCurrentLimit(10);
        rightPivotMotor.setInverted(true);

        // Initialize the left pivot motor
        leftPivotMotor = new CANSparkMax(Parameters.climber.left.PIVOT_MOTOR_ID, MotorType.kBrushless);
        leftPivotMotor.restoreFactoryDefaults();
        leftPivotMotor.enableVoltageCompensation(12);
        leftPivotMotor.setIdleMode(IdleMode.kBrake);
        leftPivotMotor.setSmartCurrentLimit(10);
        leftPivotMotor.setInverted(true);
        
        rightLimitSwitch = new DigitalInput(Parameters.climber.right.LIMIT_SWITCH_ID);
        leftLimitSwitch = new DigitalInput(Parameters.climber.left.LIMIT_SWITCH_ID);

        CANSparkMax rightMotor = new CANSparkMax(Parameters.climber.RIGHT_ID, MotorType.kBrushless);
        // leftMotor = new CANSparkMax(Parameters.climber.left.motor.ID, MotorType.kBrushless);

        // Get the encoders from the motors
        rightSpoolEncoder = rightSpoolMotor.getEncoder();
        leftSpoolEncoder = leftSpoolMotor.getEncoder();
        rightPivotEncoder = rightPivotMotor.getEncoder();
        leftPivotEncoder = leftPivotMotor.getEncoder();
        
        // Set up the encoder of the right spool motor
        rightSpoolEncoder.setPositionConversionFactor( 
                Parameters.climber.SPOOL_CIRCUMFERENCE / Parameters.climber.right.SPOOL_GEARBOX_RATIO);
        rightSpoolEncoder.setVelocityConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE / (Parameters.climber.right.SPOOL_GEARBOX_RATIO * 60));// divide once

        // Set up the encoder of the left spool motor
        leftSpoolEncoder.setPositionConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE / Parameters.climber.left.SPOOL_GEARBOX_RATIO);
        leftSpoolEncoder.setVelocityConversionFactor(
                Parameters.climber.SPOOL_CIRCUMFERENCE / (Parameters.climber.left.SPOOL_GEARBOX_RATIO * 60));// divide once


        // Set the position conversion factors
        rightPivotEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);
        leftPivotEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);



        // Set the current position of the climber to 0
        rightSpoolEncoder.setPosition(0);
        leftSpoolEncoder.setPosition(0);
        rightPivotEncoder.setPosition(0);
        leftPivotEncoder.setPosition(0);

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

    public double getRightPivotPosition(){
        return rightPivotEncoder.getPosition();
    }

    public double getLeftPivotPosition(){
        return leftPivotEncoder.getPosition();
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
        rightSpoolMotor.set(0);
        leftSpoolMotor.set(0);
        rightPivotMotor.set(0);
        leftPivotMotor.set(0);
        // leftMotor.set(0);
    }
}
