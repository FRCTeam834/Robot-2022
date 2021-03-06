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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    // Motor and motor encoder object
    CANSparkMax shooterMotor;
    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.1009, 0.357, 0.048143);
    RelativeEncoder shooterMotorEncoder;
    PIDController shooterPIDController;

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

        shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);

        // Get the encoder of the shooter motor
        shooterMotorEncoder = shooterMotor.getEncoder();

        // Set up the encoder's conversion factor
        // Multiply RPM by the circumference and 60 seconds to get m/s
        shooterMotorEncoder.setVelocityConversionFactor(
                (Parameters.shooter.WHEEL_DIA_M * Math.PI) / 60);
        shooterMotor.burnFlash();

        shooterPIDController = new PIDController(0.024089, 0, 0);
        shooterPIDController.setTolerance(0.25);
    }

    public void set(double percentage) {
        usingPID = false;
        shooterMotor.set(percentage);
    }

    public void setDesiredPID(double setpoint) {
        setVelocity = setpoint;
        usingPID = true;
    }

    public void setRPM(double rpm) {
        // shooterMotor.setRPM(rpm);
    }

    public double getSetpoint() {
        return setVelocity;
    }

    public boolean readyToShoot() {
        return shooterPIDController.atSetpoint() && RobotContainer.hood.isAtDesiredAngle();
    }

    public void stop() {
        usingPID = false;
        shooterMotor.stopMotor();
    }

    public void periodic() {
        if (usingPID) {
            shooterMotor.setVoltage(
                    shooterPIDController.calculate(shooterMotorEncoder.getVelocity(), setVelocity)
                                    * 12
                            + .9 * shooterFF.calculate(setVelocity));
        }
    }

    public double getSpeed() {
        return shooterMotorEncoder.getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Shooter");
            builder.addDoubleProperty(
                    "Setpoint",
                    shooterPIDController::getSetpoint,
                    shooterPIDController::setSetpoint);
            builder.addDoubleProperty("Measurement", shooterMotorEncoder::getVelocity, null);
            builder.addBooleanProperty("atSetpoint", shooterPIDController::atSetpoint, null);
        }
    }
}
