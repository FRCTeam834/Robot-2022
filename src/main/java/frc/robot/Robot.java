/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
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

import java.lang.reflect.Field;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
// Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Parameters.driveTrain;
import frc.robot.commands.hood.HomeHood;
import frc.robot.commands.indexing.ColorSensorIndexing;
import frc.robot.subsystems.climber.HomeClimberTubes;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private boolean linedUp;
    private boolean readyToShoot;
    public Field2d field = new Field2d();

    


    /** Moved the NavX to the Robot constructor here, allowing the NavX to only be reset once */
    Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // We don't need the telemetry
        // LiveWindow.disableAllTelemetry();

        // Reset the angle of the NavX
        RobotContainer.navX.resetYaw();
        //RobotContainer.navX.resetPitch();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();
        SmartDashboard.putData(field);
        RobotContainer.driveTrain.resetOdometry(new Pose2d());
        if (!Parameters.telemetryMode) {
            LiveWindow.disableAllTelemetry();
        }
        DataLogManager.start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        field.setRobotPose(RobotContainer.driveTrain.getEstPose2D());
        DriverStation.silenceJoystickConnectionWarning(true);
       

        // Check the state of the functions on the robot
        readyToShoot = RobotContainer.shooter.readyToShoot();
        linedUp = RobotContainer.vision.isLinedUp();

        // Decide which LED color
        if (readyToShoot && linedUp) {
            RobotContainer.lightColor = Parameters.led.GLITTER_RAINBOW;
        } else if (readyToShoot) {
            RobotContainer.lightColor = Parameters.led.OCEAN;
        } else if (linedUp) {
            RobotContainer.lightColor = Parameters.led.PINK;
        } else {
            RobotContainer.lightColor = Parameters.led.BLUE_VIOLET;
        }

        // Set the new color of the LEDs
        RobotContainer.led.set(RobotContainer.lightColor);


        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {

        // Stop all of the motors on the robot
        RobotContainer.indexer.stop();
        RobotContainer.intake.stop();
        RobotContainer.shooter.stop();
        RobotContainer.hood.stop();
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().schedule(new HomeClimberTubes(), new HomeHood());
        //new ScheduleCommand(new PerpetualCommand(new ColorSensorIndexing()));

        // Stop all of the motors on the robot
        RobotContainer.indexer.stop();
        RobotContainer.intake.stop();
        RobotContainer.shooter.stop();
        RobotContainer.hood.stop();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //System.out.println(RobotContainer.navX.getYaw());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    // Returns the color of ball that we should be collecting
    public static Color getOurBallColor() {

        // Get the alliance that we're on
        // Default to blue balls
        switch (DriverStation.getAlliance()) {
            case Red:
                return Color.kRed;
            case Blue:
                return Color.kBlue;
            default: // Used when the alliance isn't valid (not set)
                return Color.kBlue;
        }
    }
}
