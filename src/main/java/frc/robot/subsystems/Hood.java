// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  // Create motor, limit switch, etc.

    CANSparkMax hoodMotor; 
    DigitalInput limitSwitch;
    RelativeEncoder hoodEncoder;


    public Hood() {

      hoodMotor = new CANSparkMax(Parameters.hood.MOTOR_ID, MotorType.kBrushless);

      limitSwitch = new DigitalInput(Parameters.hood.LS_PORT);

      hoodEncoder = hoodMotor.getEncoder();

    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void hoodBackwards(){
        hoodMotor.set(Parameters.hood.SPEED);
    }

    public boolean getLSValue(){
        return limitSwitch.get();
    }
}
