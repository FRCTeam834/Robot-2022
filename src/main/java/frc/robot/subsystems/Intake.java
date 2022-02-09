// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Intake extends SubsystemBase {

    // Declare motor object
    CANSparkMax intakeMotor;
    Debouncer intakeCurrentDebouncer = new Debouncer(1, DebounceType.kRising);
    PowerDistribution PDP = new PowerDistribution();

    /** Creates a new Intake. */
    public Intake() {

        // Create intake motor
        intakeMotor = new CANSparkMax(Parameters.intake.motor.ID, MotorType.kBrushless);

        // Invert the direction
        intakeMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Turns on the intake
    public void intake() {
        intakeMotor.set(Parameters.intake.motor.SPEED);
    }

    // Makes the intake go in reverse
    public void spitItOut() {
        intakeMotor.set(-Parameters.intake.motor.SPEED);
    }

    // Stops the intake motors
    public void stop() {
        intakeMotor.set(0);
    }

    public boolean getIntakeCurrentSpike() {
        return intakeCurrentDebouncer.calculate(intakeMotor.getOutputCurrent() > 30);
    }
}
