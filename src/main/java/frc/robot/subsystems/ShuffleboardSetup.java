// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;


public class ShuffleboardSetup extends SubsystemBase {
  /** Creates a new ShuffleboardSetup. */

  //TODO what idk what this is someone fix!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!!!!!! - fine I'll do it myself

  private ShuffleboardTab ballRejectTab = Shuffleboard.getTab("Ball Reject");
    // ? examples of shuffleboard options 
  /*.add("My Boolean", false)
    .withWidget("Boolean Box")
    .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
    .getEntry();*/

  public ShuffleboardSetup() {}
  @Override
  public void periodic() {
    // You thought that this was a comment explaing what periodic does, but it was me, DIO!
  }
}
