// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.MovingAverage;

public class Indexer extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax indexMotor;
    RelativeEncoder indexMotorEncoder;

    // Bang-bang controller
    BangBangController bigBangTheory;

    // Color sensor object
    ColorSensorV3 colorSensor;

    // The moving average of the proximity sensor
    MovingAverage proximityAvg;

    LinearFilter proximityFilter;

    // A count of how many balls the robot has
    int ballCount = 0;

    /** Creates a new Shooter. */
    public Indexer() {

        // Create the shooter motor
        indexMotor = new CANSparkMax(Parameters.indexer.ID, MotorType.kBrushless);

        // Configure the motor's settings
        indexMotor.restoreFactoryDefaults();
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setInverted(false);
        indexMotor.setSmartCurrentLimit(Parameters.indexer.CURRENT_LIMIT);

        // Get the encoder of the shooter motor
        indexMotorEncoder = indexMotor.getEncoder();

        // Set up the color sensor
        colorSensor = new ColorSensorV3(Port.kMXP);

        // Set up the proximity average
        proximityAvg = new MovingAverage(Parameters.indexer.PROX_MOVING_AVG_PTS);
    }

    @Override
    public void periodic() {}

    public void set(double speed) {
        indexMotor.set(speed);
    }

    public void stop() {
        indexMotor.set(0);
    }

    public double getProximity() {
        return proximityAvg.addPt(colorSensor.getProximity());
    }

    public void clearProximityReadings() {
        proximityAvg.clearPts();
    }

    public boolean hasBall() {
        return (getProximity() > 105);
    }

    // For updating shuffleboard
    public String getBallColor() {

        // Get the ball color once
        Color ballColor = colorSensor.getColor();

        // If we don't have a ball, return "None"
        if (!hasBall()) {
            return "None";
        } else {

            // Check which color predominates
            if (ballColor.red > ballColor.blue) {
                return "Red";
            } else {
                // The ball must be blue
                return "Blue";
            }
        }
    }

    // Returns ballCount
    public int getBallCount() {
        return ballCount;
    }

    private double getRedColor() {
        return colorSensor.getRed();
    }

    private double getBlueColor() {
        return colorSensor.getBlue();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Indexer");
            builder.addDoubleProperty("Ball Distance", this::getProximity, null);
            builder.addDoubleProperty("Color Red Value", this::getRedColor, null);
            builder.addDoubleProperty("Color Blue Value", this::getBlueColor, null);
            builder.addBooleanProperty("Ball Detected", this::hasBall, null);
            builder.addStringProperty("Color", this::getBallColor, null);
        }
    }
}
