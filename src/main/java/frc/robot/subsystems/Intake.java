// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    // Color sensor object
    ColorSensorV3 colorSensor;

    // Color matching object
    ColorMatch colorMatcher;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.motor.ID, MotorType.kBrushless);

        // Create color sensor
        colorSensor = new ColorSensorV3(Port.kMXP);

        // Create color matching object
        colorMatcher = new ColorMatch();
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

    /**
     * Get the closest color to the readings from the color sensor
     *
     * @return The closest color. If no color is close, this will return black
     */
    public Color getClosestColor() {
        return colorMatcher.matchClosestColor(colorSensor.getColor()).color;
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