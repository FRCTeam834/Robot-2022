// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.motor.ID, MotorType.kBrushless);

        // Create color sensor

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Turns on the intake
    public void intake() {
        intakeMotor.set(Parameters.intake.motor.SPEED);
    }


    // Makes the intake go in reverse
    public void spitItOut() {
        intakeMotor.set(-Parameters.intake.motor.SPEED);
    }

    // Stops the intake motors
    public void stop() {
        intakeMotor.set(0);
    }
}
