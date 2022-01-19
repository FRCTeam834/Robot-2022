// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// You thought this was part of this top cluster of comments, but it was me, DIO!

package frc.robot.subsystems;

//Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parameters;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Encoder;


public class Shooter extends SubsystemBase {

  //construct motors
  CANSparkMax shooterMotor1 = new CANSparkMax(Parameters.shooter.ID.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(Parameters.shooter.ID.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);


  //construct a bangbang controller
  BangBangController bigBangTheory = new BangBangController();
  //! maybe add feedforward as well

  //construct a motor encode so that we can decode its encoding
  Encoder encoder = new Encoder(Parameters.shooter.ID.SHOOTER_ENCODER_IN_PORT, Parameters.shooter.ID.SHOOTER_ENCODER_OUT_PORT);

  /** Creates a new Shooter. */
  public Shooter() {
  //! May have to be somewhere else
  //set motor2 as a follower motor so they are the same speed
  shooterMotor2.follow(shooterMotor1);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void spinShooterMotors() {
    shooterMotor1.set(bigBangTheory.calculate(encoder.getRate(), Parameters.shooter.SHOOTER_SPEED));
  }

  public void spinShooterMotors(double speed){
    shooterMotor1.set(speed);
  }
}
