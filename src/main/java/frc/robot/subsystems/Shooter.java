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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Shooter extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax shooterMotor;
    RelativeEncoder shooterMotorEncoder;

    // Bang-bang controller
    BangBangController bangBangController;
    private double setPoint;

    /** Creates a new Shooter. */
    public Shooter() {

        // Create the shooter motor
        shooterMotor = new CANSparkMax(Parameters.shooter.ID, MotorType.kBrushless);

        // Configure the motor's settings
        // ! MOTOR MUST BE ON COAST FOR BANG-BANG
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setInverted(true);
        shooterMotor.setSmartCurrentLimit(Parameters.shooter.CURRENT_LIMIT);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = shooterMotor.getEncoder();

        // Set up the encoder's conversion factor
        // Multiply RPM by the circumference and 60 seconds to get m/s
        shooterMotorEncoder.setVelocityConversionFactor(
                (Parameters.shooter.WHEEL_DIA_M * Math.PI) / 60);

        // Create a new bang-bang controller
        bangBangController = new BangBangController();
        bangBangController.setTolerance(Parameters.shooter.VELOCITY_TOLERANCE);
    }

    public void set(double percentage) {
        shooterMotor.set(percentage);
    }

    public void setBangBang(double setpoint) {
        shooterMotor.set(bangBangController.calculate(shooterMotorEncoder.getVelocity(), setpoint));
    }

    // Is the motor at its setPoint
    public boolean isAtSetPoint() {
        return bangBangController.atSetpoint();
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Shooter");
            builder.addDoubleProperty(
                    "Setpoint", bangBangController::getSetpoint, bangBangController::setSetpoint);
            builder.addDoubleProperty("Measurement", shooterMotorEncoder::getVelocity, null);
            builder.addBooleanProperty("atSetpoint", this::isAtSetPoint, null);
        }
    }
}
