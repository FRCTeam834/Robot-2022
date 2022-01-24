/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808), Jadon Trackim
 *     (@JadonTrackim), Krishna Dihora (@kjdih2)
 * @since 5/8/20
 */
package frc.robot;

// Imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.DriverProfiles.ProfilingManagement;
import frc.robot.commands.swerve.StraightenWheels;
import frc.robot.commands.swerve.TurnToVision;
import frc.robot.commands.swerve.driving.LetsRoll1Joystick;
import frc.robot.commands.swerve.driving.LetsRoll2Joysticks;
import frc.robot.commands.swerve.testing.TestModulePID;
import frc.robot.commands.swerve.testing.TestModuleVelocity;
import frc.robot.commands.swerve.testing.TestMovementPID;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // subsystems
    public static ProfilingManagement profilingManagement = new ProfilingManagement();
    public static NavX navX = new NavX();
    public static DriveTrain driveTrain = new DriveTrain();
    public static Superstructure superstructure = new Superstructure(new Vision());
    // Commands
    private final LetsRoll2Joysticks letsRoll2Joysticks = new LetsRoll2Joysticks();
    private final LetsRoll1Joystick letsRoll1Joystick = new LetsRoll1Joystick();

    private final TestModulePID testModulePID = new TestModulePID();
    private final TestMovementPID testMovementPID = new TestMovementPID();
    private final TestModuleVelocity testModuleVelocity = new TestModuleVelocity();
    private final StraightenWheels straightenWheels = new StraightenWheels();
    private final TurnToVision turnToVision = new TurnToVision();

    // Define the joysticks (need to be public so commands can access axes)
    public static Joystick leftJoystick = new Joystick(0);
    public static Joystick rightJoystick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static Joystick buttonBoard = new Joystick(3);

    /*
    Button Naming Convention:
    TL = Top Left = 7
    TM = Top Middle = 2
    TR = Top Right = 4
    ML = Middle Left = 1
    MM = Middle Middle = 6
    MR = Middle Right = 3
    BL = Bottom Left = 10
    BM = Bottom Middle = 9
    BR = Bottom Right = 8
    */
    /*
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // joystick
        new JoystickButton(leftJoystick, 1).whenPressed(letsRoll2Joysticks);
        new JoystickButton(leftJoystick, 8).whenPressed(driveTrain::zeroEncoders);
        new JoystickButton(leftJoystick, 9).whenPressed(straightenWheels);
        new JoystickButton(rightJoystick, 1).whenPressed(navX::resetYaw);

        // new JoystickButton(buttonBoard,1).whenPressed(zeroNavX)
        // new JoystickButton(xbox, Button.kA.value).whenPressed(zeroNavX);
    }

    // Joystick value array, in form (LX, LY, RX, RY)
    public static double[] getJoystickValues() {

        // Create a new array to return
        double[] joystickValues = new double[4];

        // Populate the fields of the array
        joystickValues[0] = constrainJoystick(leftJoystick.getX());
        joystickValues[1] = constrainJoystick(leftJoystick.getY());
        joystickValues[2] = constrainJoystick(rightJoystick.getX());
        joystickValues[3] = constrainJoystick(rightJoystick.getY());

        // Return the array
        return joystickValues;
    }

    // Return a constrained Joystick value
    public static double constrainJoystick(double rawValue) {

        // If the value is out of tolerance, then zero it. Otherwise return the value of the
        // joystick
        if (Math.abs(rawValue) < Parameters.driver.currentProfile.joystickParams.getDeadzone()) {
            return 0;
        } else {
            switch (Parameters.driver.currentProfile.joystickParams.getOutputType()) {
                case LINEAR:
                    {
                        return rawValue;
                    }
                case ZEROED_LINEAR:
                    {
                        /**
                         * Implements the equation: output = (x - t) / (1 - t) Unfortunately, we
                         * need to deal with negative values, so we need to take the abs value, then
                         * multiply by the sign of the number Pop this into Desmos, you can see a
                         * visual output: y=\frac{x-t}{1-t}\left\{0\le y\le1\right\} Define t as a
                         * variable between 0 and 1 This equation allows the output to start at 0
                         * when leaving the threshold, then scales it so that the maximum output of
                         * the joysticks is always 1
                         */
                        return Math.signum(rawValue)
                                * ((Math.abs(rawValue)
                                                - Parameters.driver.currentProfile.joystickParams
                                                        .getDeadzone())
                                        / (1
                                                - Parameters.driver.currentProfile.joystickParams
                                                        .getDeadzone()));
                    }
                case ZEROED_QUAD:
                    {
                        /**
                         * Implements a quadratic curve, with the vertex at (t,0) and scaled to pass
                         * through the point (1,1)
                         */
                        return Math.signum(rawValue)
                                * (Math.pow(
                                                Math.abs(rawValue)
                                                        - Parameters.driver.currentProfile
                                                                .joystickParams.getDeadzone(),
                                                2)
                                        / Math.pow(
                                                Parameters.driver.currentProfile.joystickParams
                                                                .getDeadzone()
                                                        - 1,
                                                2));
                    }
                case ZEROED_QUAD_LINEAR:
                    {
                        /**
                         * Implements an output for the joysticks that uses a quadratic on the lower
                         * end and a linear slope up to 1. I'm not going to bother explaining it,
                         * here's the graph: https://www.desmos.com/calculator/5lqgnstb1k
                         */

                        // No need to implement the threshold checking, that is done above
                        if (Math.abs(rawValue)
                                < Parameters.driver.currentProfile.joystickParams
                                        .getCrossoverValue()) {

                            // This is the quadratic range, return the result of the scaled
                            // quadratic
                            return (Math.signum(rawValue)
                                    * Parameters.driver.currentProfile.joystickParams.getRampRate()
                                    * Math.pow(
                                            Math.abs(rawValue)
                                                    - Parameters.driver.currentProfile
                                                            .joystickParams.getDeadzone(),
                                            2));
                        } else {
                            // Linear equation range
                            return Math.signum(rawValue)
                                    * ((Parameters.driver.currentProfile.joystickParams
                                                            .getLinearSlope()
                                                    * (Math.abs(rawValue) - 1))
                                            + 1);
                        }
                    }
                default:
                    // This will never be reached, but a default case is needed (0 for no output)
                    return 0;
            }
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
