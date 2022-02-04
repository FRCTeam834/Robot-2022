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

public class Shooter extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax shooterMotor;
    RelativeEncoder shooterMotorEncoder;

    // Bang-bang controller
    BangBangController bigBangTheory;

    // Color sensor object
    ColorSensorV3 colorSensor;

    // Color matching object
    ColorMatch colorMatcher;


    /** Creates a new Shooter. */
    public Shooter() {

        // Create the shooter motor
        shooterMotor = new CANSparkMax(Parameters.shooter.motor.ID, MotorType.kBrushless);

        // Configure the motor's settings
        // ! MOTOR MUST BE ON COAST FOR BANG-BANG
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = shooterMotor.getEncoder();

        // Set up the encoder's conversion factor
        shooterMotorEncoder.setVelocityConversionFactor(Parameters.shooter.VEL_CONV_FACTOR);

        // Burn the flash of the Spark, prevents issues during brownouts
        shooterMotor.burnFlash();

        // Create a new bang-bang controller
        bigBangTheory = new BangBangController();

        colorSensor = new ColorSensorV3(Port.kMXP);

        // Create color matching object
        colorMatcher = new ColorMatch();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    public void set(double speed) {
        shooterMotor.set(bigBangTheory.calculate(shooterMotorEncoder.getVelocity(), speed));
    }

    public void stopMotor() {
        shooterMotor.set(0);
    }

    //! DOESN'T WORK AND IS SUPPOSED TO 
    // Returns closest color match
    public Color getClosestColor() {
        colorMatcher.setConfidenceThreshold(.5);
        return colorMatcher.matchClosestColor(colorSensor.getColor()).color;
    }

    // Determine if either red or blue is detected, if not returns neither
    public String getColorEasy(){
        if((colorSensor.getColor().red / colorSensor.getColor().blue) > 4){
            return "red";
        }
        else if((colorSensor.getColor().blue / colorSensor.getColor().red) > 4){
            return "blue";
        }
        else{
            return "neither";
        }
    }
    // return ratio red to blue
    public double ratio(){
        return colorSensor.getColor().red / colorSensor.getColor().blue;
    }
    // L + ratio + bozo + cringe + stay mad + blocked = you^∞ + co2 + c6h12o6 + small weewee
} 
