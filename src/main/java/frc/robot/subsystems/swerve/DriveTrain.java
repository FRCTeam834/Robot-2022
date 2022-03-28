/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808)
 * @since 5/8/21
 */
package frc.robot.subsystems.swerve;

// Imports
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
    /** Creates a new Drivetrain. */

    // Create the modules
    public SwerveModule frontLeft;

    public SwerveModule frontRight;
    public SwerveModule backLeft;
    public SwerveModule backRight;

    // PID value storage, with default values from Parameters
    public PIDController xMovePID = new PIDController(0, 0, 0);
    public PIDController yMovePID = new PIDController(0, 0, 0);
    public ProfiledPIDController rotPID =
            new ProfiledPIDController(2.8, 0, 0, new Constraints(Math.PI, Math.PI * 2));

    // Define module positions (relative to center of robot)
    public Translation2d FL_POS =
            new Translation2d(
                    Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    public Translation2d FR_POS =
            new Translation2d(
                    Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    public Translation2d BL_POS =
            new Translation2d(
                    -Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    public Translation2d BR_POS =
            new Translation2d(
                    -Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);

    // Create the drivetrain map
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

    // Pose estimator
    private SwerveDriveOdometry swerveDriveOdometry =
            new SwerveDriveOdometry(kinematics, RobotContainer.navX.getRotation2d());

    /** Creates a new Drivetrain object */
    public DriveTrain() {
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        // Create each swerve module instance
        frontLeft =
                new SwerveModule(
                        "FL",
                        Parameters.driveTrain.can.FL_STEER_ID,
                        Parameters.driveTrain.can.FL_DRIVE_ID,
                        Parameters.driveTrain.can.FL_CODER_ID,
                        false);
        frontRight =
                new SwerveModule(
                        "FR",
                        Parameters.driveTrain.can.FR_STEER_ID,
                        Parameters.driveTrain.can.FR_DRIVE_ID,
                        Parameters.driveTrain.can.FR_CODER_ID,
                        true);
        backLeft =
                new SwerveModule(
                        "BL",
                        Parameters.driveTrain.can.BL_STEER_ID,
                        Parameters.driveTrain.can.BL_DRIVE_ID,
                        Parameters.driveTrain.can.BL_CODER_ID,
                        false);
        backRight =
                new SwerveModule(
                        "BR",
                        Parameters.driveTrain.can.BR_STEER_ID,
                        Parameters.driveTrain.can.BR_DRIVE_ID,
                        Parameters.driveTrain.can.BR_CODER_ID,
                        true);

        // Center the odometry of the robot
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));

        // Load the offsets for the CANCoders
        loadEncoderOffsets();
    }

    /**
     * Moves the entire drivetrain with the specified X and Y velocity with rotation
     *
     * @param xVelocity X velocity, in m/s
     * @param yVelocity Y velocity, in m/s
     * @param rot Rotation velocity in rad/s
     * @param fieldRelative If true, robot will use field as X and Y reference, regardless of angle.
     *     If false, robot will move in respect to itself
     * @param tipProtection Should the drivetrain try to automatically adjust itself to prevent
     *     tipping?
     */
    public void drive(
            double xVelocity,
            double yVelocity,
            double rot,
            boolean fieldRelative,
            boolean tipProtection, boolean openLoop) {

        // Declare a variable to store the chassis speeds
        ChassisSpeeds speeds;

        // Set up the modules
        if (fieldRelative) {
            speeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity, yVelocity, rot, RobotContainer.navX.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xVelocity, yVelocity, rot);
        }

        // Check the X and Y values of the robot for tip protection
        // The chassis speeds are robot relative
        // Positive X is forward, positive Y is left
        if (tipProtection) {
            double newXSpeed =
                    adjustSpeedForAngle(RobotContainer.navX.getRoll(), speeds.vxMetersPerSecond);
            double newYSpeed =
                    adjustSpeedForAngle(RobotContainer.navX.getPitch(), speeds.vyMetersPerSecond);
            speeds = new ChassisSpeeds(newXSpeed, newYSpeed, speeds.omegaRadiansPerSecond);
        }

        // Set the modules to carry out the speeds
        setModuleStates(speeds, openLoop);
    }

    /**
     * Moves the entire drivetrain with the specified X and Y velocity with rotation (doesn't use
     * tip protection)
     *
     * @param xVelocity X velocity, in m/s
     * @param yVelocity Y velocity, in m/s
     * @param rot Rotation velocity in rad/s
     * @param fieldRelative If true, robot will use field as X and Y reference, regardless of angle.
     *     If false, robot will move in respect to itself
     */
    public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative, boolean openLoopDrive) {
        drive(xVelocity, yVelocity, rot, fieldRelative, false, openLoopDrive);
    }

    /**
     * Takes in an angle and raw speed value, and will adjust the speed to help prevent the robot
     * from tipping
     *
     * @param angle The angle of the robot, in that axis (deg)
     * @param rawValue The speed of the robot in that axis (m/s)
     * @return The adjusted speed (should help to prevent tipping)
     */
    public double adjustSpeedForAngle(double angle, double rawValue) {

        // ! STILL NEEDS TO BE TESTED
        // ! THE ROBOT COULD GET VERY ANGRY, USE EXTREME CAUTION WHEN APPROACHING
        if (Math.abs(angle) < Parameters.driver.tipProtection.MIN_TIP_ANGLE) {

            // No need to correct, we're not having an issues right now
            return rawValue;
        } else if (Math.abs(angle) > Parameters.driver.tipProtection.MAX_TIP_ANGLE) {

            // ! POOP HAS HIT THE FAN... EVASIVE MANUEVERS ACTIVATE!
            return Math.signum(angle) * -Parameters.driver.tipProtection.MAX_CORRECTION_SPEED;
        } else {
            // The angle must be between the min and maximum tip angle
            // Use a fancy ratio for the percentage of correction
            // So basically, if we're 80% to the max tip angle, when we'd
            // need to use .2 * the raw value and .8 times the correction speed
            return -Math.signum(angle)
                    * ((Math.abs(angle) - Parameters.driver.tipProtection.MIN_TIP_ANGLE)
                                    / (Parameters.driver.tipProtection.CROSSOVER_RANGE)
                                    * Parameters.driver.tipProtection.MAX_CORRECTION_SPEED
                            - (Parameters.driver.tipProtection.MAX_TIP_ANGLE - Math.abs(angle))
                                    / (Parameters.driver.tipProtection.CROSSOVER_RANGE)
                                    * rawValue);
        }
    }

    /**
     * Sets all of the states of the modules and updates the odometry of the robot
     *
     * @param chassisSpeeds The desired velocities of the movement of the entire drivetrain
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds, boolean openLoopDrive) {

        // Get the module states
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // normalizeDrive(swerveModuleStates, chassisSpeeds);
        // Scale the velocities of the swerve modules so that none exceed the maximum
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Parameters.driveTrain.maximums.MAX_TRANS_VELOCITY);

            frontLeft.setDesiredState(swerveModuleStates[0], openLoopDrive);
            frontRight.setDesiredState(swerveModuleStates[1], openLoopDrive);
            backLeft.setDesiredState(swerveModuleStates[2], openLoopDrive);
            backRight.setDesiredState(swerveModuleStates[3], openLoopDrive);

    }

    public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
        double translationalK =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                        / Parameters.driveTrain.maximums.MAX_TRANS_VELOCITY;
        double rotationalK =
                Math.abs(speeds.omegaRadiansPerSecond)
                        / Parameters.driveTrain.maximums.MAX_ROT_VELOCITY;
        double k = Math.max(translationalK, rotationalK);

        // Find the how fast the fastest spinning drive motor is spinning
        double realMaxSpeed = 0.0;
        for (SwerveModuleState moduleState : desiredStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        double scale =
                Math.min(k * Parameters.driveTrain.maximums.MAX_TRANS_VELOCITY / realMaxSpeed, 1);
        for (SwerveModuleState moduleState : desiredStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }

    /** Halts all of the modules */
    public void haltAllModules() {

        // Stop the motors of each of the modules
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Moves all of the swerve modules to the specified angles
     *
     * @param FL Angle of the front left module
     * @param FR Angle of the front right module
     * @param BL Angle of the back left module
     * @param BR Angle of the back right module
     */
    public void setDesiredAngles(double FL, double FR, double BL, double BR) {

        // Set the desired angles
        frontLeft.setDesiredAngle(FL);
        frontRight.setDesiredAngle(FR);
        backLeft.setDesiredAngle(BL);
        backRight.setDesiredAngle(BR);
    }

    /**
     * Moves the modules to the desired angles, just with an array of angles instead of individual
     * parameters
     *
     * @param angles An array of module angles in form [Front Left, Front Right, Back Left, Back
     *     Right]
     */
    public void setDesiredAngles(double[] angles) {
        setDesiredAngles(angles[0], angles[1], angles[2], angles[3]);
    }

    /**
     * Checks if the modules are at their desired angles (must all be at desired angles in order to
     * return true)
     *
     * @return Are the modules at their desired angles?
     */
    public boolean areAtDesiredAngles(double FL, double FR, double BL, double BR) {
        return (frontLeft.isAtDesiredAngle(FL)
                && frontRight.isAtDesiredAngle(FR)
                && backLeft.isAtDesiredAngle(BL)
                && backRight.isAtDesiredAngle(BR));
    }

    /**
     * Checks if the modules are at their desired angles (must all be at desired angles in order to
     * return true)
     *
     * @return Are the modules at their desired angles?
     */
    public boolean areAtDesiredAngles(double[] desiredAngles) {
        return (frontLeft.isAtDesiredAngle(desiredAngles[0])
                && frontRight.isAtDesiredAngle(desiredAngles[1])
                && backLeft.isAtDesiredAngle(desiredAngles[2])
                && backRight.isAtDesiredAngle(desiredAngles[3]));
    }

    /**
     * Sets all of the swerve modules to their specified velocities
     *
     * @param FL Velocity of the front left module
     * @param FR Velocity of the front right module
     * @param BL Velocity of the back left module
     * @param BR Velocity of the back right module
     */
    public void setDesiredVelocities(double FL, double FR, double BL, double BR) {

        // Set the modules to run at the specified velocities
        frontLeft.setDesiredVelocity(FL);
        frontRight.setDesiredVelocity(FR);
        backLeft.setDesiredVelocity(BL);
        backRight.setDesiredVelocity(BR);
    }

    /**
     * Moves the modules to the desired velocities, just with an array of velocities instead of
     * individual parameters
     *
     * @param velocities An array of module velocities in form [Front Left, Front Right, Back Left,
     *     Back Right]
     */
    public void setDesiredVelocities(double[] velocities) {
        setDesiredVelocities(velocities[0], velocities[1], velocities[2], velocities[3]);
    }

    /**
     * Checks if the modules are at their desired velocities (must all be at desired velocities in
     * order to return true)
     *
     * @return Are the modules at their desired velocities?
     */
    public boolean areAtDesiredVelocities(double FL, double FR, double BL, double BR) {
        return (frontLeft.isAtDesiredVelocity(FL)
                && frontRight.isAtDesiredVelocity(FR)
                && backLeft.isAtDesiredVelocity(BL)
                && backRight.isAtDesiredVelocity(BR));
    }

    /**
     * Checks if the modules are at their desired velocities (must all be at desired velocities in
     * order to return true)
     *
     * @return Are the modules at their desired velocities?
     */
    public boolean areAtDesiredVelocities(double[] desiredVelocities) {
        return (frontLeft.isAtDesiredVelocity(desiredVelocities[0])
                && frontRight.isAtDesiredVelocity(desiredVelocities[1])
                && backLeft.isAtDesiredVelocity(desiredVelocities[2])
                && backRight.isAtDesiredVelocity(desiredVelocities[3]));
    }

    /** Stops the drive wheel of the modules and sets it to hold stopped */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Stops the drive wheel of the modules and sets it to hold stopped */
    public void zeroVelocities() {
        setDesiredVelocities(0, 0, 0, 0);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // normalizeDrive(swerveModuleStates, chassisSpeeds);
        // Scale the velocities of the swerve modules so that none exceed the maximum
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Parameters.driveTrain.maximums.MAX_TRANS_VELOCITY);

        // Set each of the modules to their optimized state
        frontLeft.setDesiredState(desiredStates[0], false);
        frontRight.setDesiredState(desiredStates[1], false);
        backLeft.setDesiredState(desiredStates[2], false);
        backRight.setDesiredState(desiredStates[3], false);
    }

    /** Sets the modules so that they all point forward */
    public void straightenModules() {
        setDesiredAngles(0, 0, 0, 0);
    }

    /**
     * Locks the drivetrain up by halting the modules and moving them in an "X" pattern. Useful for
     * when someone is bullying us
     */
    public void lockemUp() {

        // Halt all of the motors
        // TODO: Change this to method calls instead of direct object interfacing?
        frontLeft.getDriveMotor().stopMotor();
        frontRight.getDriveMotor().stopMotor();
        backLeft.getDriveMotor().stopMotor();
        backRight.getDriveMotor().stopMotor();

        // Makes an X pattern with the swerve base
        // Set the modules to 45 degree angles
        frontLeft.setDesiredAngle(-45);
        frontRight.setDesiredAngle(45);
        backLeft.setDesiredAngle(45);
        backRight.setDesiredAngle(-45);
    }

    /** Updates the odometry. Should be called as frequently as possible to reduce error. */
    public void updateOdometry() {
        swerveDriveOdometry.update(
                RobotContainer.navX.getRotation2d(),
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
    }

    /**
     * Reset the odometry measurements. This is kind of like "homing" the robot
     *
     * @param currentPosition The robot's current position
     */
    public void resetOdometry(Pose2d currentPosition) {
        swerveDriveOdometry.resetPosition(currentPosition, RobotContainer.navX.getRotation2d());
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    public void resetOdometry(Pose2d currentPosition, Rotation2d currentAngle) {
        swerveDriveOdometry.resetPosition(currentPosition, currentAngle);
    }

    /**
     * Gets the estimated X position of the drivetrain on the field
     *
     * @return Estimated X position (m)
     */
    public double getEstXPos() {
        return swerveDriveOdometry.getPoseMeters().getX();
    }

    /**
     * Gets the estimated Y position of the drivetrain on the field
     *
     * @return Estimated Y position (m)
     */
    public double getEstYPos() {
        return swerveDriveOdometry.getPoseMeters().getY();
    }

    public void resetAllPIDControllers() {
        xMovePID.reset();
        yMovePID.reset();
        rotPID.reset(0);
    }

    public PPSwerveControllerCommand getPPSwerveContollerCommand(PathPlannerTrajectory path) {
        return new PPSwerveControllerCommand(
                path,
                this::getEstPose2D,
                kinematics,
                xMovePID,
                yMovePID,
                rotPID,
                this::setModuleStates,
                RobotContainer.driveTrain);
    }

    /**
     * Gets the estimated angle of the drivetrain on the field
     *
     * @return Estimated angle (Rotation2d)
     */
    public Rotation2d getEstAngle() {
        return swerveDriveOdometry.getPoseMeters().getRotation();
    }

    /**
     * Gets the orientation of the robot on the field
     *
     * @return The orientation of the robot (Pose2d) (units in m)
     */
    public Pose2d getEstPose2D() {
        return swerveDriveOdometry.getPoseMeters();
    }

    // Sets all of the modules to treat their current position as the zero position.
    public void zeroEncoders() {

        // Go through the CANCoders, setting each to zero
        frontLeft.setEncoderOffset(0);
        frontRight.setEncoderOffset(0);
        backLeft.setEncoderOffset(0);
        backRight.setEncoderOffset(0);
    }

    public void saveEncoderOffsets() {

        // Go through the modules, saving the encoder offsets of each one
        frontLeft.saveEncoderOffset();
        frontRight.saveEncoderOffset();
        backLeft.saveEncoderOffset();
        backRight.saveEncoderOffset();
    }

    public void loadEncoderOffsets() {

        // Go through the modules, loading the encoder offsets of each one
        frontLeft.loadEncoderOffset();
        frontRight.loadEncoderOffset();
        backLeft.loadEncoderOffset();
        backRight.loadEncoderOffset();
    }

    public void reloadSteerAngles() {

        // Go through each module, reseting their angle
        frontLeft.reloadSteerAngle();
        frontRight.reloadSteerAngle();
        backLeft.reloadSteerAngle();
        backRight.reloadSteerAngle();
    }

    public double getXSpeed() {
        return kinematics.toChassisSpeeds(
                        frontRight.getState(),
                        frontLeft.getState(),
                        backRight.getState(),
                        backLeft.getState())
                .vxMetersPerSecond;
    }

    public double getYSpeed() {
        return kinematics.toChassisSpeeds(
                        frontRight.getState(),
                        frontLeft.getState(),
                        backRight.getState(),
                        backLeft.getState())
                .vyMetersPerSecond;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (Parameters.telemetryMode) {
            builder.setSmartDashboardType("Drivetrain");
            builder.addDoubleProperty(
                    "Current X Pose: ", () -> swerveDriveOdometry.getPoseMeters().getX(), null);
            builder.addDoubleProperty(
                    "Current Y Pose: ", () -> swerveDriveOdometry.getPoseMeters().getY(), null);

            builder.addDoubleProperty("Current X Speed ", this::getXSpeed, null);

            builder.addDoubleProperty("Current X Speed: ", this::getYSpeed, null);
            builder.addDoubleProperty("Pitch", RobotContainer.navX::getPitch, null);
        }
    }

    @Override
    public void periodic() {

        // Update the odometry as frequently as possible
        updateOdometry();

        // If the tuning mode is on, check all of the PID settings
        if (Parameters.tuningMode) {
            xMovePID.setP(Parameters.driveTrain.pid.LINEAR_MOVE_P.get());
            yMovePID.setP(Parameters.driveTrain.pid.LINEAR_MOVE_P.get());
            xMovePID.setD(Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
            yMovePID.setD(Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
        }
    }
}
