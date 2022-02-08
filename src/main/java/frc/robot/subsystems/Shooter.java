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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Shooter extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax shooterMotor;
    RelativeEncoder shooterMotorEncoder;

    // Bang-bang controller
    BangBangController bigBangTheory;

    // Color sensor object
    // ColorSensorV3 colorSensor;

    // Boolean to keep track of the status of the bottom sensor in some of the intake ball methods
    Boolean sensorChanged;

    // Color matching object
    ColorMatch colorMatcher;

    // A count of how many balls the robot has
    int ballCount = 0;

    /** Creates a new Shooter. */
    public Shooter() {

        // Create the shooter motor
        shooterMotor = new CANSparkMax(Parameters.shooter.motor.ID, MotorType.kBrushless);

        // Configure the motor's settings
        // ! MOTOR MUST BE ON COAST FOR BANG-BANG
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setInverted(true);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = shooterMotor.getEncoder();

        // Set up the encoder's conversion factor
        // Multiply RPM by the circumference and 60 seconds to get m/s
        shooterMotorEncoder.setVelocityConversionFactor(
                Parameters.shooter.WHEEL_DIA_M * Math.PI * 60);

        // Create a new bang-bang controller
        bigBangTheory = new BangBangController();

        // Create color sensor (uses the MXP I2C)
        // colorSensor = new ColorSensorV3(Port.kMXP);

        // Create color matching object
        // colorMatcher = new ColorMatch();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Set the shooter motor's power
        // shooterMotor.set(bigBangTheory.calculate(shooterMotorEncoder.getVelocity()));
    }

    public void setMotorSpeed(double speed) {
        shooterMotor.set(speed);
    }
    /**
     * Sets the desired speed of the shooter
     *
     * @param speed The speed in m/s
     */
    public void setDesiredSpeed(double speed) {
        bigBangTheory.setSetpoint(speed);
    }

    public void runShooter() {
        bigBangTheory.setSetpoint(Parameters.shooter.SHOT_SPEED);
    }

    public void stop() {
        shooterMotor.set(0);
        bigBangTheory.setSetpoint(0); // This should make the bang-bang controller stop the motor
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

    public int getBallCount() {
        return ballCount;
    }

    public void setBallCount(int newValue) {
        ballCount = newValue;
    }

    public void addBallCount(int newValue) {
        ballCount += newValue;
    }

    public boolean getSensorChanged() {
        return sensorChanged;
    }

    public void setSensorChanged(boolean newValue) {
        sensorChanged = newValue;
    }
}
