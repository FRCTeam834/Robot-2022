// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.Parameters.climber.backMotor;
import frc.robot.Parameters.climber.frontMotor;

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
        frontMotor = new CANSparkMax(Parameters.climber.frontMotor.ID, MotorType.kBrushless);
        backMotor = new CANSparkMax(Parameters.climber.backMotor.ID, MotorType.kBrushless);

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
        backLimitSwitch = new DigitalInput(Parameters.climber.backMotor.LIMIT_SWITCH_CHANNEL_ID);
        frontLimitSwitch = new DigitalInput(Parameters.climber.frontMotor.LIMIT_SWITCH_CHANNEL_ID);

        // PID
        PIDController cPid = new PIDController(1, 0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean getBackLimitSwitchValue() {
        return backLimitSwitch.get();
    }

    public boolean getFrontLimitSwitchValue() {
        return frontLimitSwitch.get();
    }

    public double getFrontPosition() {
        return frontEncoder.getPosition();
    }

    public double getBackPosition() {
        return backEncoder.getPosition();
    }
    
    public double getPIDValue(double current, double setPoint){
        return cPid.calculate(current, setPoint);
        
    }

    public void setFrontMotor(double speed){
        frontMotor.set(speed);
    }

    public void setBackMotor(double speed){
        backMotor.set(speed);
    }
    
    public void stopMotors(){
        backMotor.set(0);
        frontMotor.set(0);
    }
}
