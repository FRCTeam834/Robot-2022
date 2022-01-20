// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  
  // Creates a new ColorSensor.
  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  public ColorSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Color getReading(){
    return colorSensor.getColor();
  }
}




































// ! You thought that this would be a meaningful comment, but it was me, DIO!