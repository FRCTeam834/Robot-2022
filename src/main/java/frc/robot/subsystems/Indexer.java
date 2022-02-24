// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// You thought this was part of this top cluster of comments, but it was me, DIO!

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Indexer extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax indexMotor;
    RelativeEncoder indexMotorEncoder;

    // Bang-bang controller
    BangBangController bigBangTheory;

    // Color sensor object
    ColorSensorV3 colorSensor;

    // Color matching object
    ColorMatch colorMatcher;

    // A count of how many balls the robot has
    int ballCount = 0;

    /** Creates a new Shooter. */
    public Indexer() {

        // Create the shooter motor
        indexMotor = new CANSparkMax(Parameters.indexer.ID, MotorType.kBrushless);

        // Configure the motor's settings
        indexMotor.restoreFactoryDefaults();
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setInverted(true);
        indexMotor.setSmartCurrentLimit(Parameters.indexer.CURRENT_LIMIT);

        // Get the encoder of the shooter motor
        indexMotorEncoder = indexMotor.getEncoder();

        // Set up the color sensor
        colorSensor = new ColorSensorV3(Port.kMXP);

        // Set up the color matcher
        colorMatcher = new ColorMatch();
    }

    @Override
    public void periodic() {}

    public void set(double speed) {
        indexMotor.set(speed);
    }

    public void stop() {
        indexMotor.set(0);
    }

    public boolean hasBall() {
        return (colorSensor.getProximity() > Parameters.indexer.PROXIMITY_THRESHOLD);
    }

    public boolean isRed() {

        // Get the color once, saving large amounts of time
        Color ballColor = colorSensor.getColor();

        // Decide what the color
        if ((ballColor.red / ballColor.blue) > 2.5) {
            return true;
        } else if ((ballColor.blue / ballColor.red) > 2.5) {
            return false;
        } else {
            return false;
        }
    }

    // For updating shuffleboard
    public String getBallColorString() {

        Color ballColor = colorSensor.getColor();

        if ((ballColor.red / ballColor.blue) > 2.5) {
            return "Red";
        } else if ((ballColor.blue / ballColor.red) > 2.5) {
            return "Blue";
        } else {
            return "None";
        }
    }
    // Returns ballCount
    public int getBallCount() {
        return ballCount;
    }
}
