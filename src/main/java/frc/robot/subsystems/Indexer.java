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
        proximityAvg = new MovingAverage(Parameters.indexer.MOVING_AVG_PTS);
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
        return (getProximity() > Parameters.indexer.PROXIMITY_THRESHOLD);
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

        if ((ballColor.red > ballColor.blue) && hasBall()) {
            return "Red";
        } else if ((ballColor.blue > ballColor.red) && hasBall()) {
            return "Blue";
        } else {
            return "None";
        }
    }

    // Returns ballCount
    public int getBallCount() {
        return ballCount;
    }

    public double getRedColor() {
        return colorSensor.getRed();
    }

    public double getBlueColor() {
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
            builder.addStringProperty("Color", this::getBallColorString, null);
        }
    }
}
