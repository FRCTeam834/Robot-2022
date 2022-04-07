// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// You thought this was part of this top cluster of comments, but it was me, DIO!

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.commands.shooting.ShootBalls;
import frc.robot.utilityClasses.TuneableNumber;

public class Shooter extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax shooterMotor;
    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.1009, 0.31, 0.048143);
    RelativeEncoder shooterMotorEncoder;
    PIDController shooterPIDController;
    BangBangController shooterBangBangController = new BangBangController();
    TuneableNumber shooterP;

    // Store if we're using PID
    boolean usingPID = false;

    // Store the set velocity
    double setVelocity = 0;

    /** Creates a new Shooter. */
    public Shooter() {

        // Create the shooter motor
        shooterMotor = new CANSparkMax(Parameters.shooter.ID, MotorType.kBrushless);

        // Configure the motor's settings
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setInverted(false);
        shooterMotor.setSmartCurrentLimit(Parameters.shooter.CURRENT_LIMIT);

        // Reduce the status frame updates for position (we only use the velocity reading, so it
        // would just clog up the readings)
        shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = shooterMotor.getEncoder();

        // Set up the encoder's conversion factor
        // Multiply RPM by the circumference and 60 seconds to get m/s
        shooterMotorEncoder.setVelocityConversionFactor(
                (Parameters.shooter.WHEEL_DIA_M * Math.PI) / 60);
        shooterMotor.burnFlash();

        shooterPIDController = new PIDController(0.065, 0, 0);
        shooterPIDController.setTolerance(.1);
        shooterBangBangController.setTolerance(.1);
    }

    public void setP(double p) {
        shooterPIDController.setP(p);
    }

    public void set(double percentage) {
        usingPID = false;
        shooterMotor.set(percentage);
    }

    public void setDesiredSpeed(double setpoint) {
        setVelocity = setpoint;
        usingPID = true;
    }

    // WARNING: THIS IS DUMB, ONLY ESTIMATES MOTOR OUTPUT
    public void setRPM(double rpm) {
        usingPID = false;
        shooterMotor.set(rpm / 5820);
    }

    public double getSetpoint() {
        return setVelocity;
    }

    public boolean isReady() {
        return shooterBangBangController.atSetpoint() && usingPID;
    }

    public void stop() {
        usingPID = false;
        shooterMotor.stopMotor();
    }

    public void periodic() {
        if (usingPID) {
            // shooterMotor.setVoltage(
            //       shooterPIDController.calculate(shooterMotorEncoder.getVelocity(), setVelocity)
            //                      * 12
            //             + shooterFF.calculate(setVelocity));
            shooterMotor.setVoltage(
                    shooterBangBangController.calculate(
                                    shooterMotorEncoder.getVelocity(), setVelocity)
                            + shooterFF.calculate(setVelocity));
        }
    }

    public double getSpeed() {
        return shooterMotorEncoder.getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Shooter");
            builder.addDoubleProperty("Setpoint", shooterBangBangController::getSetpoint, null);
            // builder.addDoubleProperty("Shooter P", shooterPIDController::getP, this::setP);
            builder.addDoubleProperty(
                    "Measurement", shooterMotorEncoder::getVelocity, this::setDesiredSpeed);
            // builder.addDoubleProperty("Shooter P", shooterPIDController::getP, this::setP);
            builder.addDoubleProperty(
                    "Measurement", shooterMotorEncoder::getVelocity, this::setDesiredSpeed);
            builder.addBooleanProperty("atSetpoint", shooterBangBangController::atSetpoint, null);
            builder.addDoubleProperty(
                    "Time Since Last Ball", ShootBalls.timeSinceLastIndexedBall::get, null);
        }
    }
}
