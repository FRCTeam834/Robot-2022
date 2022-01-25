// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    /** Creates a new Superstructure. */

    // Declare our subsystems
    public static Vision vision;

    // public static Hood hood;
    // public static Conveyor conveyor;
    public Superstructure(Vision vision) {
        Superstructure.vision = vision;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}