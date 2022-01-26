// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;

public class Climber extends SubsystemBase {

    // Motor objects
    CANSparkMax frontMotor;
    CANSparkMax backMotor;

    // Encoder objects (from NEOs)
    public RelativeEncoder frontEncoder;
    public RelativeEncoder backEncoder;

    // Limit Switch
    DigitalInput frontLimitSwitch;
    DigitalInput backLimitSwitch;
    private PIDController cPid;

    /** Creates a new Climber. */
    public Climber() {

        // Create the motors
        // frontMotor = new CANSparkMax(Parameters.climber.frontMotor.ID, MotorType.kBrushless);
        // backMotor = new CANSparkMax(Parameters.climber.backMotor.ID, MotorType.kBrushless);

        // Get the encoders from the motors
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();

        // Enable voltage compensation
        frontMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);
        backMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);

        // Set the position conversion factors
        frontEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);
        backEncoder.setPositionConversionFactor(Parameters.climber.POS_CONV_FACTOR);

        // Set the current position of the climber to 0
        // ! This means that the climber must start at the every bottom every time!
        frontEncoder.setPosition(0);
        backEncoder.setPosition(0);

        // Create the limit switches
        // backLimitSwitch = new DigitalInput(Parameters.climber.backMotor.LIMIT_SWITCH_CHANNEL_ID);
        // frontLimitSwitch = new
        // DigitalInput(Parameters.climber.frontMotor.LIMIT_SWITCH_CHANNEL_ID);

        // PID
        PIDController cPid = new PIDController(1, 0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // Unravels spools simultaneuously to extend both arms at the same time
    public void extendClimber(double speed, double setPoint) {
        do {
            if (frontEncoder.getPosition() < backEncoder.getPosition()) {
                frontMotor.set(
                        cPid.calculate(frontEncoder.getPosition(), backEncoder.getPosition()));
            } else if (frontEncoder.getPosition() > backEncoder.getPosition()) {
                backMotor.set(
                        cPid.calculate(backEncoder.getPosition(), frontEncoder.getPosition()));
            } else {
                frontMotor.set(speed);
                backMotor.set(speed);
            }

        } while (frontEncoder.getPosition() < setPoint || backEncoder.getPosition() < setPoint);
    }

    public void doAPullUp(double speed) {
        /*
        do {
            if (frontEncoder.getPosition() < backEncoder.getPosition()) {
                frontMotor.set(
                        cPid.calculate(frontEncoder.getPosition(), backEncoder.getPosition()));
            } else if (frontEncoder.getPosition() > backEncoder.getPosition()) {
                backMotor.set(
                        cPid.calculate(backEncoder.getPosition(), frontEncoder.getPosition()));
            } else {
                frontMotor.set(speed);
                backMotor.set(speed);
            }

        } while (frontEncoder.getPosition() < setPoint || backEncoder.getPosition() < setPoint);*/
    }

    public boolean getBackLimitSwitchValue() {
        return backLimitSwitch.get();
    }

    public boolean getFrontLimitSwitchValue() {
        return frontLimitSwitch.get();
    }

    public double getFrontEncoder() {
        return frontEncoder.getPosition();
    }
}
