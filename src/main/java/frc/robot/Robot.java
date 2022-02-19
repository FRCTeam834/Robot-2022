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

import edu.wpi.first.math.geometry.Pose2d;
// Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private boolean shooterAtSpeed;
    private boolean linedUp;

    /** Moved the NavX to the Robot constructor here, allowing the NavX to only be reset once */
    Robot() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // We don't need the telemetry
        // LiveWindow.disableAllTelemetry();

        // Reset the angle of the NavX
        RobotContainer.navX.resetYaw();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        RobotContainer.driveTrain.resetOdometry(new Pose2d());
        RobotContainer.led.set(RobotContainer.lightColor);
        // Runs HomeIntake and HomeHood commands
        // CommandScheduler.getInstance()
        //        .schedule(RobotContainer.getHomeIntake(), RobotContainer.getHomeHood());
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     *
     * <p>You thought this was a comment that would explain about this function, but it was me, DIO!
     */
    @Override
    public void robotPeriodic() {

        // Check the state of the functions on the robot
        shooterAtSpeed = RobotContainer.shooter.isAtSetPoint();
        linedUp = RobotContainer.vision.isLinedUp();

        // Decide which LED color
        if (shooterAtSpeed && linedUp) {
            RobotContainer.lightColor = Parameters.led.GLITTER_RAINBOW;
        } else if (shooterAtSpeed) {
            RobotContainer.lightColor = Parameters.led.OCEAN;
        } else if (linedUp) {
            RobotContainer.lightColor = Parameters.led.PINK;
        } else {
            RobotContainer.lightColor = Parameters.led.BLUE_VIOLET;
        }

        // Set the new color of the LEDs
        RobotContainer.led.set(RobotContainer.lightColor);

        //System.out.println(RobotContainer.intakeSpool.getSpoolPosition());
        // System.out.println(String.format("S: %.2f | A: %.2f",
        // Units.radiansToDegrees(RobotContainer.driveTrain.rotationPID.getSetpoint().position),
        // RobotContainer.navX.getYaw()));

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
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

        // Stop all of the motors on the robot
        RobotContainer.indexer.stop();
        RobotContainer.intake.stop();
        RobotContainer.shooter.stop();
        RobotContainer.hood.stop();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

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
