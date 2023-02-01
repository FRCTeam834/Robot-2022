// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    /** Creates a new Intake. */
    private final CANSparkMax intakeMotor;

    public final RelativeEncoder intakeEncoder;
    private final SparkMaxPIDController controller;
    public final LinearFilter filter;
    private boolean isStalled;

    public Intake() {
        intakeMotor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        filter = LinearFilter.movingAverage(12);
        intakeMotor.setSmartCurrentLimit(20);
        controller = intakeMotor.getPIDController();
        controller.setP(0.003);
        controller.setI(0.0);
        controller.setD(0.0);
    }

    public boolean isMotorStalled() {
        return isStalled;
    }

    public void startMotorForward() {
        // intakeMotor.set(1);
        controller.setReference(12, CANSparkMax.ControlType.kVoltage);
    }

    public void startMotorBackward() {
        // intakeMotor.set(-1);
        controller.setReference(-12, CANSparkMax.ControlType.kVoltage);
    }

    public void stopMotor() {
        intakeMotor.set(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Better Intake");
        builder.addDoubleProperty("Current", intakeMotor::getOutputCurrent, null);
    }

    @Override
    public void periodic() {}
}
