/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup), Mohammad Durrani (@mdurrani808), Jadon Trackim
 *     (@JadonTrackim), Krishna Dihora (@kjdih2)
 * @since 5/8/20
 */
package frc.robot;

// Imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmMotorDown;
import frc.robot.commands.ArmMotorUp;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.StopArmMotor;
import frc.robot.commands.swerve.driving.LetsRoll;
import frc.robot.commands.swerve.driving.LetsRollEgoCentric;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.swerve.DriveTrain;
import frc.robot.utilityClasses.ButtonBoard;


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
    public static Intake intake = new Intake();

    public static Arm arm = new Arm();

    // Commands
    // Normally we can just declare a new object for the command, but we need to keep track of if
    // the intake command isn't running

    // Auton chooser
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Define the joysticks (need to be public so commands can access axes)
    public static Joystick leftJoystick = new Joystick(0);
    public static Joystick rightJoystick = new Joystick(1);
    public static XboxController xbox = new XboxController(2);
    public static Joystick buttonBoard = new Joystick(3);

    // Define button board buttons
    public static JoystickButton TL = new JoystickButton(buttonBoard, ButtonBoard.TL);
    public static JoystickButton TM = new JoystickButton(buttonBoard, ButtonBoard.TM);
    public static JoystickButton TR = new JoystickButton(buttonBoard, ButtonBoard.TR);
    public static JoystickButton ML = new JoystickButton(buttonBoard, ButtonBoard.ML);
    public static JoystickButton MM = new JoystickButton(buttonBoard, ButtonBoard.MM);
    public static JoystickButton MR = new JoystickButton(buttonBoard, ButtonBoard.MR);
    public static JoystickButton BL = new JoystickButton(buttonBoard, ButtonBoard.BL);
    public static JoystickButton BM = new JoystickButton(buttonBoard, ButtonBoard.BM);
    public static JoystickButton BR = new JoystickButton(buttonBoard, ButtonBoard.BR);

    // If the robot should be field centric
    public static boolean fieldCentric = Parameters.driver.fieldCentric;

    // The robot's turn rate
    public static double turnRate = Parameters.driver.slowSteerRate;

    // If the indexer should be running (autoschedules)
    public static boolean autoIndex = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        SmartDashboard.putData(autoChooser);
        if (Parameters.telemetryMode) {
            SmartDashboard.putData(navX);
            SmartDashboard.putData(driveTrain);
        }

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

        // Default commands
        // Automatically run the swerve when not shooting
        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.driveTrain, new LetsRoll());

        
        // Left Joystick
        // new JoystickButton(leftJoystick, 1)
        //        .whenPressed(
        //                new InstantCommand(
        //                        () -> RobotContainer.fieldCentric =
        // !RobotContainer.fieldCentric));
        new JoystickButton(leftJoystick, 1).whenPressed(new IntakeCone());

        new JoystickButton(leftJoystick, 2).whenPressed(new ArmMotorUp(arm));
        new JoystickButton(leftJoystick, 3).whenPressed(new ArmMotorDown(arm));
        new JoystickButton(leftJoystick, 4).whenPressed(new StopArmMotor(arm));
        

        // new JoystickButton(leftJoystick, 2).whenPressed(new LetsRoll());
        // new JoystickButton(leftJoystick, 3).whenPressed(new InstantCommand(navX::resetYaw));
        // new JoystickButton(leftJoystick, 4).whenPressed(new LetsRollEgoCentric());
        new JoystickButton(leftJoystick, 8)
                .and(new JoystickButton(leftJoystick, 9))
                .whenActive(
                        new InstantCommand(driveTrain::zeroEncoders, driveTrain)
                                .andThen(
                                        new PrintCommand("Zeroed!")
                                                .andThen(
                                                        new InstantCommand(
                                                                driveTrain::saveEncoderOffsets,
                                                                driveTrain))));

        // Right Joystick
        /*new JoystickButton(rightJoystick, 1)
        .whenPressed(
                new InstantCommand(
                        () ->
                                RobotContainer.turnRate =
                                        (RobotContainer.turnRate
                                                        == Parameters.driver.fastSteerRate)
                                                ? Parameters.driver.slowSteerRate
                                                : Parameters.driver.fastSteerRate));*/
        // () -> RobotContainer.fieldCentric = !RobotContainer.fieldCentric));
        // new JoystickButton(rightJoystick, 2).whenPressed(new WaitForShooter());
        // new JoystickButton(rightJoystick, 3).whenPressed(() -> driveTrain.reloadSteerAngles());
        // new JoystickButton(rightJoystick, 4).whenPressed(new HomeIntake());

        // right and left lift up
        /*BM.whenPressed(
        new ParallelCommandGroup(
                new MoveTubeToPosition(
                        RobotContainer.climbers2.leftLift,
                        (Parameters.climber.lift.UP_LEGAL_DISTANCE_LEFT),
                        1),
                new MoveTubeToPosition(
                        RobotContainer.climbers2.rightLift,
                        (Parameters.climber.lift.UP_LEGAL_DISTANCE_RIGHT),*/

        // right and lift down

        // right and left tilt up

        // Runs function tests
        // Holding down keeps the test running, letting go cycles to the next on the next button
        // push
        // new JoystickButton(rightJoystick, 10).whileHeld(new FunctionTest());
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
        if (Math.abs(rawValue) < Parameters.driver.controllers.deadzone) {
            return 0;
        } else {
            switch (Parameters.driver.controllers.clampingType) {
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
                                * ((Math.abs(rawValue) - Parameters.driver.controllers.deadzone)
                                        / (1 - Parameters.driver.controllers.deadzone));
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
                                                        - Parameters.driver.controllers.deadzone,
                                                2)
                                        / Math.pow(Parameters.driver.controllers.deadzone - 1, 2));
                    }
                default:
                    // This will never be reached, but a default case is needed (0 for no output)
                    return 0;
            }
        }
    }

    /** Homes all of the robot's PID controllers if they haven't already been homed */
    public void homeAllPIDControllers() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
