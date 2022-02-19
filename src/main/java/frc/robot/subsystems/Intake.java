// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;

    // Motor object for spool
    CANSparkMax spoolMotor;

    // Motor encoder
    RelativeEncoder spoolMotorEncoder;

    // spool PID controller
    CachedPIDController pidController;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the spool has been homed yet
    boolean homed = false;

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

        // Invert the direction
        intakeMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(Parameters.intake.INTAKE_MOTOR_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
    }

    // Turns on the intake
    public void turnOn() {
        intakeMotor.set(Parameters.intake.INTAKE_SPEED);
    }


    // Makes the intake go in reverse
    public void spitItOut() {
        intakeMotor.set(-Parameters.intake.INTAKE_SPEED);
    }

    // Stops the intake motors
    public void stop() {
        intakeMotor.set(0);
    }

    
}
