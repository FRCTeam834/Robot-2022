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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.swerve.LetsRoll1Joystick;
import frc.robot.commands.swerve.LetsRoll2Joysticks;
import frc.robot.commands.swerve.PullNTSwerveParams;
import frc.robot.commands.swerve.SaveSwerveParameters;
import frc.robot.commands.swerve.StraightenWheels;
import frc.robot.commands.swerve.TestModulePID;
import frc.robot.commands.swerve.TestModuleVelocity;
import frc.robot.commands.swerve.TestMovementPID;
import frc.robot.commands.swerve.ZeroCanCoders;
import frc.robot.commands.swerve.ZeroNavX;
import frc.robot.enums.RobotState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // Subsystems
    // private final ProfilingManagement profilingManagement = new ProfilingManagement();
    // private final NavX navX = new NavX();
    // private final DriveTrain driveTrain = new DriveTrain();
    // private final UltrasonicSensor ultrasonicSensor = new UltrasonicSensor();

    // Commands
    private final LetsRoll2Joysticks letsRoll2Joysticks = new LetsRoll2Joysticks();
    private final LetsRoll1Joystick letsRoll1Joystick = new LetsRoll1Joystick();
    private final ZeroCanCoders zeroCanCoders = new ZeroCanCoders();
    private final PullNTSwerveParams pullNtSwerveParams = new PullNTSwerveParams();
    private final TestModulePID testModulePID = new TestModulePID();
    private final TestMovementPID testMovementPID = new TestMovementPID();
    private final TestModuleVelocity testModuleVelocity = new TestModuleVelocity();
    private final SaveSwerveParameters saveSwerveParameters = new SaveSwerveParameters();
    private final ZeroNavX zeroNavX = new ZeroNavX();
    private final StraightenWheels straightenWheels = new StraightenWheels();

    // Timer (for delays)
    public static Timer timer = new Timer();

    // Define the joysticks (need to be public so commands can access axes)
    public static Joystick leftJoystick = new Joystick(0);
    public static Joystick rightJoystick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);

    // Left Joystick button array
    // public static JoystickButton leftJoystickButtons[];

    // Right Joystick button array
    // public static JoystickButton rightJoystickButtons[];

    // The robot's state
    public static RobotState robotState;

    public static final JoystickButton
            // Left Joystick
            lJoystick1 = new JoystickButton(leftJoystick, 1),
            lJoystick2 = new JoystickButton(leftJoystick, 2),
            lJoystick3 = new JoystickButton(leftJoystick, 3),
            lJoystick4 = new JoystickButton(leftJoystick, 4),
            lJoystick5 = new JoystickButton(leftJoystick, 5),
            lJoystick6 = new JoystickButton(leftJoystick, 6),
            lJoystick7 = new JoystickButton(leftJoystick, 7),
            lJoystick8 = new JoystickButton(leftJoystick, 8),
            lJoystick9 = new JoystickButton(leftJoystick, 9),
            lJoystick10 = new JoystickButton(leftJoystick, 10),
            lJoystick11 = new JoystickButton(leftJoystick, 11),

            // Right Joystick
            rJoystick1 = new JoystickButton(rightJoystick, 1),
            rJoystick2 = new JoystickButton(rightJoystick, 2),
            rJoystick3 = new JoystickButton(rightJoystick, 3),
            rJoystick4 = new JoystickButton(rightJoystick, 4),
            rJoystick5 = new JoystickButton(rightJoystick, 5),
            rJoystick6 = new JoystickButton(rightJoystick, 6),
            rJoystick7 = new JoystickButton(rightJoystick, 7),
            rJoystick8 = new JoystickButton(rightJoystick, 8),
            rJoystick9 = new JoystickButton(rightJoystick, 9),
            rJoystick10 = new JoystickButton(rightJoystick, 10),
            rJoystick11 = new JoystickButton(rightJoystick, 11);

    // Arcade Buttons
    /*
    Button Naming Convention:
    BG = Button Group
    TL = Top Left
    TM = Top Middle
    TR = Top Right
    ML = Middle Left
    MM = Middle Middle
    MR = Middle Right
    BL = Bottom Left
    BM = Bottom Middle
    BR = Bottom Right
    */
    /*
    BGTL = new JoystickButton(launchpad, 7), BGTM = new JoystickButton(launchpad, 2),
    BGTR = new JoystickButton(launchpad, 4), BGML = new JoystickButton(launchpad, 1),
    BGMM = new JoystickButton(launchpad, 6),
    BGMR = new JoystickButton(launchpad, 3),
    BGBL = new JoystickButton(launchpad, 10), BGBM = new JoystickButton(launchpad, 9),
    BGBR = new JoystickButton(launchpad, 8);
    */

    // Xbox Buttons
    /*
    private final JoystickButton xboxStart = new JoystickButton(xbox, Button.kStart.value), xboxBack = new JoystickButton(xbox, Button.kBack.value),
    xboxB = new JoystickButton(xbox, Button.kB.value), xboxA = new JoystickButton(xbox, Button.kA.value), xboxY = new JoystickButton(xbox, Button.kY.value),
    xboxX = new JoystickButton(xbox, Button.kX.value), xboxLB = new JoystickButton(xbox, Button.kBumperLeft.value), xboxRB = new JoystickButton(xbox, Button.kBumperRight.value),
    xboxLJB = new JoystickButton(xbox, 9); */

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

        // Left joystick
        lJoystick1.whenPressed(letsRoll2Joysticks);
        // lJoystick2.whenPressed(saveSwerveParameters);
        // lJoystick3.whenPressed(pullNtSwerveParams);
        // lJoystick4.whenPressed(testModulePID);
        // lJoystick5.whenPressed(testMovementPID);
        lJoystick8.whenPressed(zeroCanCoders);
        lJoystick9.whenPressed(straightenWheels);

        // Right joystick
        rJoystick1.toggleWhenPressed(zeroNavX);

        /*
        // Try to assign the left joystick
        try {
          leftJoystick = new Joystick(0);

          // If we get here, then the left joystick was successful and we can try the right joystick
          try {
            rightJoystick = new Joystick(1);

            // Both joysticks are present
            robotState = ROBOT_STATE.TWO_JOYSTICKS;

          }
          catch (Exception e) {

            // We only have one joystick, the left
            robotState = ROBOT_STATE.ONE_JOYSTICK;
          }

        }
        catch (Exception e) {

          // No joysticks detected
          robotState = ROBOT_STATE.NO_JOYSTICKS;
        }


        // Setup the robot based on the state of it
        if (robotState == ROBOT_STATE.TWO_JOYSTICKS) {
          // Full setup

          // Left Joystick button assignment (buttons array starts at 0)
          for(int buttonIndex = 0; buttonIndex < Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
            leftJoystickButtons[buttonIndex] = new JoystickButton(leftJoystick, buttonIndex);
          }

          // Right Joystick button assignment (buttons array starts at 0)
          for(int buttonIndex = 0; buttonIndex < Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
            rightJoystickButtons[buttonIndex] = new JoystickButton(rightJoystick, buttonIndex);
          }

          // Command setup
          // Configure the command (on the second button of the joystick)

        }
        else if (robotState == ROBOT_STATE.ONE_JOYSTICK) {
          // Left Joystick button assignment (buttons array starts at 0)
          for(int buttonIndex = 1; buttonIndex <= Parameters.joysticks.JOYSTICK_BUTTON_COUNT; buttonIndex++) {
            leftJoystickButtons[buttonIndex - 1] = new JoystickButton(leftJoystick, buttonIndex);
          }

          // Command setup
          // Configure the command (on the second button of the joystick)
          leftJoystickButtons[0].whenPressed(letsRoll1Joystick);
          leftJoystickButtons[1].whenPressed(saveSwerveParameters);
          leftJoystickButtons[2].whenPressed(pullNtSwerveParams);
          //leftJoystickButtons[3].whenPressed(testModulePID);
          leftJoystickButtons[4].whenPressed(testMovementPID);
          leftJoystickButtons[7].whenPressed(zeroCanCoders);
        }
        else {
          // No joysticks (show mode)

          // Command setup

        }
        */

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
