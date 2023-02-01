// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class FunctionTest extends CommandBase {

    // The current thing that we're testing
    private static int testNumber = 0;

    // The total number of tests
    private static final int maxTestNumber = 5;

    /** Creates a new FunctionTest. */
    public FunctionTest() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(
);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

  

        // Increment the test number (so that we move to the next test)
        testNumber++;

        // Use the modulus so that the test number never exceeds the number of tests
        testNumber %= maxTestNumber;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // This should never be true, as it uses a while held
        return false;
    }
}
