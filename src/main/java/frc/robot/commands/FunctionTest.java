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
                RobotContainer.driveTrain,
                RobotContainer.hood,
                RobotContainer.indexer,
                RobotContainer.shooter,
                RobotContainer.climbers2,
                RobotContainer.intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Decide what to run
        switch (testNumber) {
            case 0: // Intake
                RobotContainer.intake.set(Parameters.intake.INTAKE_SPEED);
                break;
            case 1:
                RobotContainer.indexer.set(Parameters.indexer.FEED_SPEED);
                break;
            default:
                System.out.println("TEST NUMBER EXCEEDED MAXIMUM");
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Shuts off the motors after the command is done
        // ! EXTREMELY IMPORTANT!
        switch (testNumber) {
            case 0:
                RobotContainer.intake.stop();
                break;
            case 1:
                RobotContainer.indexer.stop();
            default:
                System.out.println("TEST NUMBER EXCEEDED MAXIMUM");
        }

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
