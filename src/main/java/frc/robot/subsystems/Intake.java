// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate.Param;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Parameters;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  //create intake motor!!!!!!
  CANSparkMax intakeMotorBottom = new CANSparkMax(Parameters.driveTrain.can.INTAKEMOTORBOTTOM_ID, MotorType.kBrushless);
  CANSparkMax intakeMotorTop = new CANSparkMax(Parameters.driveTrain.can.INTAKEMOTORTOP_ID, MotorType.kBrushless);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnOn(){
    intakeMotorTop.set(Parameters.intake.INTAKEMOTORTOP_SPEED);
    intakeMotorBottom.set(Parameters.intake.INTAKEMOTORBOTTOM_SPEED);

  }
}
