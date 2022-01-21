// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Parameters;

public class Climber extends SubsystemBase {

  //fun different comment colors are among us?
  // ? Hello
  // * Hello
  // TODO: Hello
  // ! You thought that it was another hello comment, but it me me, DIO!

  // ! creates a motor| CHANGE NAME WHEN WE HAVE A CLIMBER DESIGN
  CANSparkMax climberMotor1 = new CANSparkMax(Parameters.climber.can.CLIMB_MOTOR_1_ID, MotorType.kBrushless);
  CANSparkMax climberMotor2 = new CANSparkMax(Parameters.climber.can.CLIMB_MOTOR_2_ID, MotorType.kBrushless);

  //TODO creates a limit switch if needed

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void firstMotorSpin() {
    climberMotor1.set(Parameters.climber.CLIMB_MOTOR_1_SPEED);
  }

  public void firstMotorSpin(double speed){
    climberMotor1.set(speed);
  }

  public void spinBothMotors(){
    climberMotor1.set(Parameters.climber.CLIMB_MOTOR_1_SPEED);
    climberMotor2.set(Parameters.climber.CLIMB_MOTOR_2_SPEED);
  }

  public void spinBothMotors(double speed1, double speed2){
    climberMotor1.set(speed1);
    climberMotor2.set(speed2);
  }
}
