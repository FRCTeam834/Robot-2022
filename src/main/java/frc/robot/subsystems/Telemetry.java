// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;


public class Telemetry extends SubsystemBase {
  /** Creates a new Telemetry. */

  public Telemetry() {
      // Create shuiffleboard buttons(input) and data jawns here(output)
      SmartDashboard.putNumber("Ball Count", (double) RobotContainer.indexer.getBallCount());
      SmartDashboard.putString("Next Ball Color", RobotContainer.indexer.getBallColorString());
      Shuffleboard.getTab("Indexer Data");
    
      
  }

  // Updates the values in shuffleboard
  public void update(){
    SmartDashboard.putNumber("Ball Count", (double) RobotContainer.indexer.getBallCount());
    SmartDashboard.putString("Next Ball Color", RobotContainer.indexer.getBallColorString());
    


      

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }
}
