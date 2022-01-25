// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// You thought this was part of this top cluster of comments, but it was me, DIO!

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    public void spinShooterMotor(double speed) {
        shooterMotor.set(bigBangTheory.calculate(shooterMotorEncoder.getVelocity(), speed));
    }

    public void stopMotor() {
        shooterMotor.set(0);
    }
}
