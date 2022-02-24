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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.autons.DriveForwardAuton;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.indexing.ColorSensorIndexing;
import frc.robot.commands.intake.HomeIntake;
import frc.robot.commands.intake.SwitchIntakeState;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.commands.swerve.StraightenWheels;
import frc.robot.commands.swerve.TurnToAngleVision;
import frc.robot.commands.swerve.driving.LetsRoll;
import frc.robot.commands.swerve.testing.TestModulePID;
import frc.robot.commands.swerve.testing.TestModulePositioning;
import frc.robot.commands.swerve.testing.TestModuleVelocity;
import frc.robot.commands.swerve.testing.TestRotationalPID;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWinch;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.HomeClimberTubes;
import frc.robot.subsystems.swerve.DriveTrain;
import frc.robot.utilityClasses.ButtonBoard;
import frc.robot.utilityClasses.interpolation.InterpolatingTable;

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
    public static Hood hood = new Hood();

    public static Climber climber = new Climber();
    public static Intake intake = new Intake();
    public static IntakeWinch intakeWinch = new IntakeWinch();
    public static Shooter shooter = new Shooter();
    public static Indexer indexer = new Indexer();
    public static Vision vision = new Vision();
    public static InterpolatingTable interpolatingTable = new InterpolatingTable();

    // Commands

    // Movement
    private final LetsRoll letsRoll = new LetsRoll();

    // Debugging
    private final TestModulePID testModulePID = new TestModulePID();
    private final TestModulePositioning testModulePositioning = new TestModulePositioning();
    private final TestRotationalPID testRotationalPID = new TestRotationalPID();
    private final TestModuleVelocity testModuleVelocity = new TestModuleVelocity();
    private final StraightenWheels straightenWheels = new StraightenWheels();

    // Intaking/holding balls
    private final ColorSensorIndexing indexingThings = new ColorSensorIndexing();
    private final SwitchIntakeState switchIntakeState = new SwitchIntakeState();

    // Homing commands
    private final HomeHood homeHood = new HomeHood();
    private final HomeIntake homeIntake = new HomeIntake();
    private final HomeClimberTubes homeClimberTubes = new HomeClimberTubes();

    // Autoshooting
    private final TurnToAngleVision turnToGoal = new TurnToAngleVision();
    private final AutoShoot autoShoot = new AutoShoot();

    // Lights! No camera and no action
    public static Spark led = new Spark(Parameters.led.PORT);
    public static double lightColor = -.45;

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        SmartDashboard.putData(shooter);
        SmartDashboard.putData(hood);
        SmartDashboard.putData(navX);
        SmartDashboard.putData(vision);
        SmartDashboard.putData(driveTrain);
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
        new JoystickButton(leftJoystick, 1)
                .whenPressed(
                        new InstantCommand(
                                () -> RobotContainer.fieldCentric = !RobotContainer.fieldCentric));
        new JoystickButton(leftJoystick, 2).whenPressed(letsRoll);
        new JoystickButton(leftJoystick, 3).whenPressed(new InstantCommand(navX::resetYaw));
        new JoystickButton(leftJoystick, 8)
                .whenPressed(
                        new InstantCommand(driveTrain::zeroEncoders, driveTrain)
                                .andThen(new PrintCommand("Zeroed!")));
        // new JoystickButton(leftJoystick, 9).whenPressed();

        // Right Joystick
        new JoystickButton(rightJoystick, 1)
                .whenPressed(
                        new InstantCommand(
                                () ->
                                        RobotContainer.turnRate =
                                                (RobotContainer.turnRate
                                                                == Parameters.driver.fastSteerRate)
                                                        ? Parameters.driver.slowSteerRate
                                                        : Parameters.driver.fastSteerRate));
        new JoystickButton(rightJoystick, 2).whenPressed(autoShoot);

        // Button board
        BM.whileHeld(
                new InstantCommand(
                        () ->
                                shooter.setDesiredSpeed(
                                        rightJoystick.getZ() * Parameters.shooter.MAX_SPEED)));
        BR.whenPressed(new InstantCommand(() -> shooter.stop()));
        BL.whileHeld(
                new InstantCommand(
                        () ->
                                hood.setDesiredAngle(
                                        leftJoystick.getZ() * (Parameters.hood.ALLOWABLE_RANGE)
                                                + Parameters.hood.HOME_ANGLE)));
        TL.whenPressed(
                new InstantCommand(
                        () ->
                                interpolatingTable.addEntry(
                                        vision.getDistanceToGoal(),
                                        hood.getDesiredAngle(),
                                        shooter.getDesiredSpeed())));
        TM.whenPressed(new InstantCommand(intake::turnOn, intake));
        TR.whenPressed(new InstantCommand(intake::stop, intake));
        MM.whenPressed(new InstantCommand(() -> indexer.set(0.35), indexer));
        MR.whenPressed(new InstantCommand(() -> indexer.set(0)));
        TL.whenPressed(switchIntakeState);

        /*
        // run the hood down (inlined)
        new JoystickButton(xbox, Button.kLeftBumper.value)
                .whenHeld(
                        new StartEndCommand(() -> hood.runMotor(.05), hood::stop, hood)
                                .withInterrupt(hood::getLSValue));

        // run the hood up (inlined)
        new JoystickButton(xbox, Button.kRightBumper.value)
                .whenHeld(new StartEndCommand(() -> hood.runMotor(-.05), hood::stop, hood));

        // intake balls (inlined)
        new JoystickButton(xbox, Button.kY.value)
                .whenHeld(new StartEndCommand(intake::turnOn, intake::stop, intake));
        */
        // index balls (inlined)
        new JoystickButton(xbox, Button.kA.value).whenPressed(new DriveForwardAuton());

        new JoystickButton(xbox, Button.kB.value)
                .whenPressed(
                        new InstantCommand(driveTrain::zeroEncoders, driveTrain)
                                .andThen(
                                        new PrintCommand("Zeroed!")
                                                .andThen(
                                                        new InstantCommand(
                                                                driveTrain::saveEncoderOffsets,
                                                                driveTrain))));

        new JoystickButton(xbox, Button.kY.value)
                .whenPressed(() -> driveTrain.setDesiredAngles(0, 0, 0, 0));
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

    /** Homes all of the robot's PID controllers if they haven't already been homed */
    public void homeAllPIDControllers() {

        // Check each if each is homed, running homing if not
        if (!hood.isHomed()) {
            CommandScheduler.getInstance().schedule(false, homeHood);
        }
        if (!intakeWinch.isHomed()) {
            CommandScheduler.getInstance().schedule(false, homeIntake);
        }
        if (!climber.areTubesHomed()) {
            CommandScheduler.getInstance().schedule(false, homeClimberTubes);
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
