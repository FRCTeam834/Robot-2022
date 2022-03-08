// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    // Homing limit switch
    DigitalInput limitSwitch;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Invert the direction
        intakeMotor.setInverted(false);

        // Set the current limit
        intakeMotor.setSmartCurrentLimit(Parameters.intake.INTAKE_MOTOR_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {}

    // Turns on the intake
    public void set(double percentage) {
        intakeMotor.set(percentage);
    }

    // Stops the intake motors
    public void stop() {
        intakeMotor.set(0);
    }
}
