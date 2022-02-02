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
    public void suckItIn() {
        intakeMotor.set(Parameters.intake.motor.SPEED);
    }

    // Run the intake till a ball is sucked in, then return the color
    public Color suckABall() {

        // Create a variable to store the ball's color
        Color ballColor;

        // Start the intake
        suckItIn();

        // Wait for a ball in the color sensor
        do {
            ballColor = getClosestColor();
        } while (!ballColor.equals(Color.kRed) && !ballColor.equals(Color.kBlue));

        // Stop the intake
        stop();

        // Return the found color
        return ballColor;
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
