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


public class Shooter extends SubsystemBase {

  CANSparkMax shooterMotor1 = new CANSparkMax(Parameters.driveTrain.can.SHOOTERMOTOR1_ID, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(Parameters.driveTrain.can.SHOOTERMOTOR2_ID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void spinShooterMotors() {
    shooterMotor1.set(Parameters.shooter.SHOOTERMOTOR1_SPEED);
    shooterMotor2.set(Parameters.shooter.SHOOTERMOTOR2_SPEED);
  }

  public void spinShooterMotors(double speed1, double speed2){
    shooterMotor1.set(speed1);
    shooterMotor2.set(speed2);
  }
}
