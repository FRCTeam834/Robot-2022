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
    CANSparkMax frontMotor;
    CANSparkMax backMotor;

    // Encoder objects (from NEOs)
    RelativeEncoder frontEncoder;
    RelativeEncoder backEncoder;

    /** Creates a new Climber. */
    public Climber() {

        // Create the motors
        frontMotor = new CANSparkMax(Parameters.climber.frontMotor.ID, MotorType.kBrushless);
        backMotor = new CANSparkMax(Parameters.climber.backMotor.ID, MotorType.kBrushless);

        // Get the encoders from the motors
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();

        // Enable voltage compensation
        frontMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);
        backMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);

        // Set the position conversion factors
        frontEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);
        backEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);

        // Set the current position of the climber to 0
        // ! This means that the climber must start at the every bottom every time!
        frontEncoder.setPosition(0);
        backEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extendClimber() {}
}
