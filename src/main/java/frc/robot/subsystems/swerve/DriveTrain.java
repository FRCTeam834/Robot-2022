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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
    /** Creates a new Drivetrain. */

    // Create the modules
    public SwerveModule frontLeft;

    public SwerveModule frontRight;
    public SwerveModule backLeft;
    public SwerveModule backRight;

    public Field2d field = new Field2d();
    // PID value storage, with default values from Parameters
    public PIDController xMovePID =
            new PIDController(
                    Parameters.driveTrain.pid.LINEAR_MOVE_P.get(),
                    Parameters.driveTrain.pid.LINEAR_MOVE_I,
                    Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
    public PIDController yMovePID =
            new PIDController(
                    Parameters.driveTrain.pid.LINEAR_MOVE_P.get(),
                    Parameters.driveTrain.pid.LINEAR_MOVE_I,
                    Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
    public ProfiledPIDController rotationPID =
            new ProfiledPIDController(
                    Parameters.driveTrain.pid.ROT_MOVE_P.get(),
                    Parameters.driveTrain.pid.ROT_MOVE_I,
                    Parameters.driveTrain.pid.ROT_MOVE_D.get(),
                    new Constraints(
                            Units.degreesToRadians(
                                    Parameters.driveTrain.pid.DEFAULT_ROT_MAX_VELOCITY),
                            Units.degreesToRadians(
                                    Parameters.driveTrain.pid.DEFAULT_ROT_MAX_ACCEL)));

    // Define module positions (relative to center of robot)
    Translation2d FL_POS =
            new Translation2d(
                    Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    Translation2d FR_POS =
            new Translation2d(
                    Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    Translation2d BL_POS =
            new Translation2d(
                    -Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);
    Translation2d BR_POS =
            new Translation2d(
                    -Parameters.driveTrain.dimensions.DRIVE_LENGTH / 2,
                    -Parameters.driveTrain.dimensions.DRIVE_WIDTH / 2);

    // Create the drivetrain map
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

    // Pose estimator
    private SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(kinematics, RobotContainer.navX.getRotation2d());

    // Holomonic drive controller
    private HolonomicDriveController driveController =
            new HolonomicDriveController(xMovePID, yMovePID, rotationPID);

    /** Creates a new Drivetrain object */
    public DriveTrain() {

        SmartDashboard.putData("Field", field);
        // Create each swerve module instance
        frontLeft =
                new SwerveModule(
                        "FL",
                        Parameters.driveTrain.can.FL_STEER_ID,
                        Parameters.driveTrain.can.FL_DRIVE_ID,
                        Parameters.driveTrain.can.FL_CODER_ID,
                        true);
        frontRight =
                new SwerveModule(
                        "FR",
                        Parameters.driveTrain.can.FR_STEER_ID,
                        Parameters.driveTrain.can.FR_DRIVE_ID,
                        Parameters.driveTrain.can.FR_CODER_ID,
                        false);
        backLeft =
                new SwerveModule(
                        "BL",
                        Parameters.driveTrain.can.BL_STEER_ID,
                        Parameters.driveTrain.can.BL_DRIVE_ID,
                        Parameters.driveTrain.can.BL_CODER_ID,
                        true);
        backRight =
                new SwerveModule(
                        "BR",
                        Parameters.driveTrain.can.BR_STEER_ID,
                        Parameters.driveTrain.can.BR_DRIVE_ID,
                        Parameters.driveTrain.can.BR_CODER_ID,
                        false);

        // Set up the PID controllers
        rotationPID.setTolerance(Parameters.driveTrain.pid.DEFAULT_ROT_TOLERANCE);

        // Center the odometry of the robot
        resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
    }

    /**
     * Moves the entire drivetrain with the specified X and Y velocity with rotation
     *
     * @param xVelocity X velocity, in m/s
     * @param yVelocity Y velocity, in m/s
     * @param rot Rotation velocity in rad/s
     * @param fieldRelative If true, robot will use field as X and Y reference, regardless of angle.
     *     If false, robot will move in respect to itself
     */
    public void drive(double xVelocity, double yVelocity, double rot, boolean fieldRelative) {

        // Set up the modules
        if (fieldRelative) {
            setModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity, yVelocity, rot, RobotContainer.navX.getRotation2d()));
        } else {
            setModuleStates(new ChassisSpeeds(xVelocity, yVelocity, rot));
        }
    }

    /**
     * Moves the entire drivetrain with specified X and Y velocity with rotation around a specified
     * relative center
     *
     * @param xVelocity X velocity, in m/s
     * @param yVelocity Y velocity, in m/s
     * @param rot Rotation velocity in rad/s
     * @param fieldRelative If true, robot will use field as X and Y reference, regardless of angle.
     *     If false, robot will move in respect to itself
     * @param relativeCenter A Translation2d of the point that the robot is supposed to move around.
     *     This point is relative to the frame of the robot
     */
    public void driveRelativeCenter(
            double xVelocity,
            double yVelocity,
            double rot,
            boolean fieldRelative,
            Translation2d relativeCenter) {

        // Drive with selected mode
        if (fieldRelative) {
            setModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity, yVelocity, rot, RobotContainer.navX.getRotation2d()),
                    relativeCenter);
        } else {
            setModuleStates(new ChassisSpeeds(xVelocity, yVelocity, rot), relativeCenter);
        }
    }

    /**
     * Moves the entire drivetrain with specified X and Y velocity with rotation around a specified
     * absolute center
     *
     * @param xVelocity X velocity, in m/s
     * @param yVelocity Y velocity, in m/s
     * @param rot Rotation velocity in rad/s
     * @param fieldRelative If true, robot will use field as X and Y reference, regardless of angle.
     *     If false, robot will move in respect to itself
     * @param relativeCenter A Translation2d of the point that the robot is supposed to move around.
     *     This point is relative to the field
     */
    public void driveAbsoluteCenter(
            double xVelocity,
            double yVelocity,
            double rot,
            boolean fieldRelative,
            Translation2d absoluteCenter) {

        // Get the current position of the robot on the field
        Pose2d currentPose = swerveDriveOdometry.getPoseMeters();

        // Create a pose of the field coordinate
        Pose2d fieldCenterPose = new Pose2d(absoluteCenter, new Rotation2d(0));

        // Convert the field coordinates to robot coordinates
        Translation2d centerOfRotation = fieldCenterPose.relativeTo(currentPose).getTranslation();

        // Run the modules using the relative position that was just calculated
        driveRelativeCenter(xVelocity, yVelocity, rot, fieldRelative, centerOfRotation);
    }

    /**
     * Sets all of the states of the modules and updates the odometry of the robot
     *
     * @param chassisSpeeds The desired velocities of the movement of the entire drivetrain
     * @param centerOfRotation The center of rotation that should be used. This is relative to the
     *     robot
     */
    private void setModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {

        // Get the module states
        SwerveModuleState[] swerveModuleStates =
                kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

        // Scale the velocities of the swerve modules so that none exceed the maximum
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Parameters.driveTrain.maximums.MAX_MODULE_VELOCITY);

        // Set each of the modules to their optimized state
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets all of the states of the modules and updates the odometry of the robot
     *
     * @param chassisSpeeds The desired velocities of the movement of the entire drivetrain
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {

        // Get the module states
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Scale the velocities of the swerve modules so that none exceed the maximum
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Parameters.driveTrain.maximums.MAX_MODULE_VELOCITY);

        // Set each of the modules to their optimized state
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
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
    public void stopModules() {
        setDesiredVelocities(0, 0, 0, 0);
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
    }
    public void resetOdometry(Pose2d currentPosition, Rotation2d currentAngle)
    {
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

    /**
     * Check if the robot is at the reference point of the planned movement
     *
     * @return Is the movement finished?
     */
    public boolean finishedMovement() {
        return driveController.atReference();
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

    /**
     * Calculates the next angular speed for the drivetrain
     *
     * @param desiredAngle the final angle desired (in deg) (using an absolute angle)
     * @return The correction output from the controller (in deg/s)
     */
    public double turnToAbsoluteAngle(double desiredAngle) {

        // Calculate the next output, returning the feedforward result it returns
        // Unfortunately, WPI uses radians under the hood, which means that we have to use radians
        // as well
        double radiansPerSec =
                rotationPID.calculate(
                        RobotContainer.navX.getRotation2d().getRadians(),
                        Units.degreesToRadians(desiredAngle));
        return Units.radiansToDegrees(radiansPerSec);
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
    public void periodic() {

        // Update the odometry as frequently as possible
        updateOdometry();

        field.setRobotPose(swerveDriveOdometry.getPoseMeters());

        SmartDashboard.putNumber("Current X Pose: ", swerveDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Current Y Pose: ", swerveDriveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber(
                "Current Rotation: ",
                swerveDriveOdometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Current X Speed: ", getXSpeed());
        SmartDashboard.putNumber("Current Y Speed: ", getYSpeed());
        SmartDashboard.putNumber(
                "Current Gyro Angle: ", RobotContainer.navX.getRotation2d().getDegrees());

        
        // If the tuning mode is on, check all of the PID settings
        if (Parameters.tuningMode) {
            xMovePID.setP(Parameters.driveTrain.pid.LINEAR_MOVE_P.get());
            yMovePID.setP(Parameters.driveTrain.pid.LINEAR_MOVE_P.get());
            xMovePID.setD(Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
            yMovePID.setD(Parameters.driveTrain.pid.LINEAR_MOVE_D.get());
            rotationPID.setP(Parameters.driveTrain.pid.ROT_MOVE_P.get());
            rotationPID.setD(Parameters.driveTrain.pid.ROT_MOVE_D.get());
        }
    }
}
