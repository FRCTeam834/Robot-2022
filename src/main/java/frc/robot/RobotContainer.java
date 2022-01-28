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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Parameters.driveTrain.pid.drive;
import frc.robot.commands.swerve.StraightenWheels;
import frc.robot.commands.swerve.driving.LetsRoll2Joysticks;
import frc.robot.commands.swerve.testing.TestModulePID;
import frc.robot.commands.swerve.testing.TestModulePositioning;
import frc.robot.commands.swerve.testing.TestModuleVelocity;
import frc.robot.commands.swerve.testing.TestMovementPID;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.swerve.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // Subsystems
    public static NavX navX = new NavX();
    public static DriveTrain driveTrain = new DriveTrain();
    // public static Superstructure superstructure = new Superstructure(new Vision());
    public static Climber climber = new Climber();
    // public static Intake intake = new Intake();
    // public static Shooter shooter = new Shooter();

    // Commands
    private final LetsRoll2Joysticks letsRoll2Joysticks = new LetsRoll2Joysticks();
    private final TestModulePID testModulePID = new TestModulePID();
    private final TestMovementPID testMovementPID = new TestMovementPID();
    private final TestModulePositioning testModulePositioning = new TestModulePositioning();
    private final TestModuleVelocity testModuleVelocity = new TestModuleVelocity();
    private final StraightenWheels straightenWheels = new StraightenWheels();
    // private final TurnToVision turnToVision = new TurnToVision();

    // Define the joysticks (need to be public so commands can access axes)
    public static Joystick leftJoystick = new Joystick(0);
    public static Joystick rightJoystick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static Joystick buttonBoard = new Joystick(3);

    // Button Board Naming Convention Variables:
    private static final int BBTL = 7;  // Top Left
    private static final int BBTM = 2;  // Top Middle
    private static final int BBTR = 4;  // Top Right
    private static final int BBML = 1;  // Middle Left
    private static final int BBMM = 6;  // Middle Middle
    private static final int BBMR = 3;  // Middle Right
    private static final int BBBL = 10; // Bottom Left
    private static final int BBBM = 9;  // Bottom Middle
    private static final int BBBR = 8;  // Bottom Right

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

        // Left Joystick
        new JoystickButton(leftJoystick, 1).whenPressed(letsRoll2Joysticks);
        new JoystickButton(leftJoystick, 2).whenPressed(testModulePositioning);
        new JoystickButton(leftJoystick, 8)
                .whenPressed(
                        new InstantCommand(driveTrain::zeroEncoders, driveTrain)
                                .andThen(new PrintCommand("Zeroed!")));
        new JoystickButton(leftJoystick, 9).whenPressed(straightenWheels);

        // Right Joystick
        new JoystickButton(rightJoystick, 1).whenPressed(new InstantCommand(navX::resetYaw, navX).andThen(new PrintCommand("Reset NavX!")));
        new JoystickButton(rightJoystick, 8).whenPressed(new InstantCommand(driveTrain::saveParameters, driveTrain).andThen(new PrintCommand("Saved tunings!")));
        new JoystickButton(rightJoystick, 9).whenPressed(new InstantCommand(driveTrain::defaultParameters, driveTrain).andThen(new PrintCommand("Defaulted tunings!")));

        // Button board
        new JoystickButton(buttonBoard, BBTR).whenPressed(new InstantCommand(climber::runRightMotor, climber));
        new JoystickButton(buttonBoard, BBTR).whenReleased(new InstantCommand(climber::stopMotors, climber));
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
        if (Math.abs(rawValue) < Parameters.driver.joysticks.deadzone) {
            return 0;
        } else {
            switch (Parameters.driver.joysticks.clampingType) {
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
                                * ((Math.abs(rawValue) - Parameters.driver.joysticks.deadzone)
                                        / (1 - Parameters.driver.joysticks.deadzone));
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
                                                        - Parameters.driver.joysticks.deadzone,
                                                2)
                                        / Math.pow(Parameters.driver.joysticks.deadzone - 1, 2));
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
