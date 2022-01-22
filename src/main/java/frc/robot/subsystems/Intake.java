// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Parameters;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  //create intake motors!!!!!!
  CANSparkMax intakeMotor = new CANSparkMax(Parameters.intake.can.INTAKE_MOTOR, MotorType.kBrushless);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //turns on the intake
  public void setForward(){
    intakeMotor.set(Parameters.intake.INTAKE_MOTOR_TOP_SPEED);
  }

  //Makes the intake go in reverse
  public void setReverse(){
    intakeMotor.set(-Parameters.intake.INTAKE_MOTOR_SPEED);
  }

  //Stops the intake motors
  public void stop(){
    intakeMotor.set(0);

  }
}
