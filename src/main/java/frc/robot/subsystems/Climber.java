// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Parameters;

public class Climber extends SubsystemBase {

  //fun different comment colors are among us?
  // ? Hello
  // * Hello
  // TODO: Hello
  // ! You thought that it was another hello comment, but it me me, DIO!

  CANSparkMax frontMotor = new CANSparkMax(Parameters.climber.can.FRONT_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax backMotor = new CANSparkMax(Parameters.climber.can.BACK_MOTOR_ID, MotorType.kBrushless);

  CANCoder frontCANCoder = new CANCoder(Parameters.climber.can.FRONT_MOTOR_ID);
  CANCoder backCANCoder = new CANCoder(Parameters.climber.can.BACK_MOTOR_ID);
  

  //TODO creates a limit switch if needed
  DigitalInput frontLimitSwitch = new DigitalInput(0);


  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void extendClimber(){

  } 


}
