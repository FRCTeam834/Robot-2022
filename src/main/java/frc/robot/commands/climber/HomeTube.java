// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.subsystems.climber.TelescopingTube;

public class HomeTube extends CommandBase {

    // The tube to be homed
    TelescopingTube tube;

    // The homing speed (in duty percent)
    double homeSpeed;

    // The homed position
    double homeDistance;

    /** Creates a new HomeTube. */
    public HomeTube(TelescopingTube tube, double homeSpeed, double homeDistance) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(tube);

        // Save the tube that we're interacting with
        this.tube = tube;

        // Save the speed we should be moving at
        this.homeSpeed = homeSpeed;

        // Save the distance of the home position
        this.homeDistance = homeDistance;

        this.tube.setCurrentLimit(Parameters.climber.TUBE_HOME_CURRENT);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!tube.getLSValue()) {
            tube.set(homeSpeed);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the motor (VERY IMPORTANT!)
        tube.stop();
        // Set motor current limit back to normal
        this.tube.setCurrentLimit(Parameters.climber.TUBE_CURRENT_LIMIT);

        // Set that the tube is homed (if it wasn't interrupted)
        if (!interrupted) {
            tube.setCurrentDistance(homeDistance);
            tube.setDesiredDistance(homeDistance);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return tube.getLSValue();
    }
}
