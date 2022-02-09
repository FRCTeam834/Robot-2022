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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Indexer extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax indexMotor;
    RelativeEncoder shooterMotorEncoder;

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
        indexMotor = new CANSparkMax(Parameters.indexer.motor.ID, MotorType.kBrushless);

        // Configure the motor's settings
        indexMotor.restoreFactoryDefaults();
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setInverted(true);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = indexMotor.getEncoder();

        // Set up the color sensor
        colorSensor = new ColorSensorV3(Port.kMXP);

        // Set up the color matcher
        colorMatcher = new ColorMatch();
    }

    @Override
    public void periodic() {

    }

    public void setMotorSpeed(double speed) {
        indexMotor.set(speed);
    }

    public void stop() {
        indexMotor.set(0);
    }

    public boolean hasBall() {
        return (colorSensor.getProximity() > Parameters.indexer.colorSensor.PROXIMITY_THRESHOLD);
    }

    public boolean isRed() {
        if ((colorSensor.getColor().red / colorSensor.getColor().blue) > 2.5) {
            return true;
        } else if ((colorSensor.getColor().blue / colorSensor.getColor().red) > 2.5) {
            return false;
        } else {
            return false;
        }
    }
    /*
    // ! DOESN'T WORK
    // Returns closest color match
    public Color getClosestColor() {
        colorMatcher.setConfidenceThreshold(.5);
        return colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    }

    // Determine if either red or blue is detected, if not returns neither
    public Color getColor() {
        if ((colorSensor.getColor().red / colorSensor.getColor().blue) > 4) {
            return Color.kRed;
        } else if ((colorSensor.getColor().blue / colorSensor.getColor().red) > 4) {
            return Color.kBlue;
        } else {
            return Color.kBlack;
        }
    }

    // Return ratio of red to blue
    public double getRedBlueRatio() {
        return colorSensor.getColor().red / colorSensor.getColor().blue;
    }

    // For top sensor: if there is an object close returns true if theres not returns false
    public boolean getTopSensor() {
        if (colorSensor.getProximity() > 1800) {
            return true;
        } else {
            return false;
        }
    }*/

}
