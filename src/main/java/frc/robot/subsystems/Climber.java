// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Climber extends SubsystemBase {

    // Motor objects
    CANSparkMax rightMotor;
    // CANSparkMax leftMotor;

    // Encoder objects (from NEOs)
    private RelativeEncoder rightEncoder;
    // private RelativeEncoder leftEncoder;

    // Limit Switch
    // DigitalInput rightBottomLimitSwitch;
    // DigitalInput leftBottomLimitSwitch;

    /** Creates a new Climber. */
    public Climber() {

        // Create the motors
        rightMotor = new CANSparkMax(Parameters.climber.right.motor.ID, MotorType.kBrushless);
        // leftMotor = new CANSparkMax(Parameters.climber.left.motor.ID, MotorType.kBrushless);

        // Get the encoders from the motors
        rightEncoder = rightMotor.getEncoder();
        // leftEncoder = leftMotor.getEncoder();

        // Enable voltage compensation
        rightMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);
        // leftMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);

        // Set the position conversion factors
        rightEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);
        // leftEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);

        // Set the current position of the climber to 0
        // ! This means that the climber must start at the every bottom every time!
        rightEncoder.setPosition(0);
        // leftEncoder.setPosition(0);

        // Create the limit switches
        // frontBottomLimitSwitch = new DigitalInput(Parameters.climber.front.limitSwitch.DIO_CHAN);
        // backBottomLimitSwitch = new DigitalInput(Parameters.climber.back.limitSwitch.DIO_CHAN);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    /*public double getLeftPosition() {
       return leftEncoder.getPosition();
    }*/

    public void setRightMotor(double speed) {
        rightMotor.set(speed);
    }

    public void runRightMotor() {
        rightMotor.set(Parameters.climber.DEFAULT_SPEED);
    }

    /*public void setLeftMotor(double speed) {
        leftMotor.set(speed);
    }*/

    public void stopMotors() {
        rightMotor.set(0);
        // leftMotor.set(0);
    }
}
