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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
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
    public PIDController X_MOVE_PID = new PIDController(
            Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_P,
            Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_I,
            Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_D);
    public PIDController Y_MOVE_PID = new PIDController(
        Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_P,
        Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_I,
        Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_D);
    public ProfiledPIDController ROTATION_PID = new ProfiledPIDController(
        Parameters.driveTrain.pid.DEFAULT_ROT_MOVE_P,
        Parameters.driveTrain.pid.DEFAULT_ROT_MOVE_I,
        Parameters.driveTrain.pid.DEFAULT_ROT_MOVE_D,
        new Constraints(
                Units.degreesToRadians(Parameters.driveTrain.pid.DEFAULT_ROT_MAX_VELOCITY),
                Units.degreesToRadians(Parameters.driveTrain.pid.DEFAULT_ROT_MAX_ACCEL)));

    // NetworkTable entries
    NetworkTableEntry X_MOVE_PID_P_ENTRY;
    NetworkTableEntry X_MOVE_PID_I_ENTRY;
    NetworkTableEntry X_MOVE_PID_D_ENTRY;
    NetworkTableEntry Y_MOVE_PID_P_ENTRY;
    NetworkTableEntry Y_MOVE_PID_I_ENTRY;
    NetworkTableEntry Y_MOVE_PID_D_ENTRY;
    NetworkTableEntry ROTATION_PID_P_ENTRY;
    NetworkTableEntry ROTATION_PID_I_ENTRY;
    NetworkTableEntry ROTATION_PID_D_ENTRY;
    NetworkTableEntry ROTATION_PID_MAX_ACCEL_ENTRY;
    NetworkTableEntry ROTATION_PID_MAX_VEL_ENTRY;
    NetworkTableEntry X_POSITION_ENTRY;
    NetworkTableEntry Y_POSITION_ENTRY;
    NetworkTableEntry ROTATIONAL_POSITION_ENTRY;

    // Define their position (relative to center of robot)
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
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    RobotContainer.navX.getRotation2d(),
                    new Pose2d(0.0, 0.0, new Rotation2d()),
                    kinematics,
                    Parameters.driveTrain.movement.POSE_STD_DEV,
                    Parameters.driveTrain.movement.ENCODER_GYRO_DEV,
                    Parameters.driveTrain.movement.VISION_DEVIATION);

    // Holomonic drive controller
    private HolonomicDriveController driveController =
            new HolonomicDriveController(X_MOVE_PID, Y_MOVE_PID, ROTATION_PID);

    /** Creates a new Drivetrain object */
    public DriveTrain() {

        // Create each swerve module instance
        frontLeft =
                new SwerveModule(
                        "FL",
                        Parameters.driveTrain.can.FL_STEER_ID,
                        Parameters.driveTrain.can.FL_DRIVE_ID,
                        Parameters.driveTrain.can.FL_CODER_ID,
                        Parameters.driveTrain.pid.FL_STEER_PID,
                        Parameters.driveTrain.pid.FL_DRIVE_PID,
                        true);
        frontRight =
                new SwerveModule(
                        "FR",
                        Parameters.driveTrain.can.FR_STEER_ID,
                        Parameters.driveTrain.can.FR_DRIVE_ID,
                        Parameters.driveTrain.can.FR_CODER_ID,
                        Parameters.driveTrain.pid.FR_STEER_PID,
                        Parameters.driveTrain.pid.FR_DRIVE_PID,
                        false);
        backLeft =
                new SwerveModule(
                        "BL",
                        Parameters.driveTrain.can.BL_STEER_ID,
                        Parameters.driveTrain.can.BL_DRIVE_ID,
                        Parameters.driveTrain.can.BL_CODER_ID,
                        Parameters.driveTrain.pid.BL_STEER_PID,
                        Parameters.driveTrain.pid.BL_DRIVE_PID,
                        true);
        backRight =
                new SwerveModule(
                        "BR",
                        Parameters.driveTrain.can.BR_STEER_ID,
                        Parameters.driveTrain.can.BR_DRIVE_ID,
                        Parameters.driveTrain.can.BR_CODER_ID,
                        Parameters.driveTrain.pid.BR_STEER_PID,
                        Parameters.driveTrain.pid.BR_DRIVE_PID,
                        false);

        // Set up the PID controllers
        ROTATION_PID.setTolerance(Parameters.driveTrain.pid.DEFAULT_ROT_TOLERANCE);

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Set up the module's table on NetworkTables
            NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
            NetworkTable driveTrainTable = swerveTable.getSubTable("DRIVETRAIN");

            // PID tables
            NetworkTable xPIDTable = driveTrainTable.getSubTable("X_PID");
            NetworkTable yPIDTable = driveTrainTable.getSubTable("Y_PID");
            NetworkTable rotationPIDTable = driveTrainTable.getSubTable("ROTATION_PID");
            NetworkTable positionTable = driveTrainTable.getSubTable("POSITION");

            // Create new entries for the PID tuning values
            // X movement
            X_MOVE_PID_P_ENTRY = xPIDTable.getEntry("P");
            X_MOVE_PID_I_ENTRY = xPIDTable.getEntry("I");
            X_MOVE_PID_D_ENTRY = xPIDTable.getEntry("D");

            // Y movement
            Y_MOVE_PID_P_ENTRY = yPIDTable.getEntry("P");
            Y_MOVE_PID_I_ENTRY = yPIDTable.getEntry("I");
            Y_MOVE_PID_D_ENTRY = yPIDTable.getEntry("D");

            // Rotational movement
            ROTATION_PID_P_ENTRY = rotationPIDTable.getEntry("P");
            ROTATION_PID_I_ENTRY = rotationPIDTable.getEntry("I");
            ROTATION_PID_D_ENTRY = rotationPIDTable.getEntry("D");
            ROTATION_PID_MAX_ACCEL_ENTRY = rotationPIDTable.getEntry("MAX_ACCEL");
            ROTATION_PID_MAX_VEL_ENTRY = rotationPIDTable.getEntry("MAX_VEL");

            // Position data
            X_POSITION_ENTRY = positionTable.getEntry("X");
            Y_POSITION_ENTRY = positionTable.getEntry("Y");
            ROTATIONAL_POSITION_ENTRY = positionTable.getEntry("THETA");

            // Push the parameters to NetworkTables
            publishTuningValues();
        }

        // Load the saved parameters from memory
        loadParameters();

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
        Pose2d currentPose = poseEstimator.getEstimatedPosition();

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
    private void setModuleStates(ChassisSpeeds chassisSpeeds) {

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
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();
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
    public boolean areAtDesiredAngles() {
        return (frontLeft.isAtDesiredAngle()
                && frontRight.isAtDesiredAngle()
                && backLeft.isAtDesiredAngle()
                && backRight.isAtDesiredAngle());
    }

    /**
     * Sets all of the swerve modules to their specified velocities
     *
     * @param FL Velocity of the front left module
     * @param FR Velocity of the front right module
     * @param BL Velocity of the back left module
     * @param BR Velocity of the back right module
     */
    public void setDesiredVelocities(
            double FL, double FR, double BL, double BR) {

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
     * @param velocities An array of module velocities in form [Front Left, Front Right, Back
     *     Left, Back Right]
     */
    public void setDesiredVelocities(double[] velocities) {
        setDesiredVelocities(
                velocities[0], velocities[1], velocities[2], velocities[3]);
    }

    /**
     * Checks if the modules are at their desired velocities (must all be at desired velocities in
     * order to return true)
     *
     * @return Are the modules at their desired velocities?
     */
    public boolean areAtDesiredVelocities() {
        return (frontLeft.isAtDesiredVelocity()
                && frontRight.isAtDesiredVelocity()
                && backLeft.isAtDesiredVelocity()
                && backRight.isAtDesiredVelocity());
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
        frontLeft.driveMotor.stopMotor();
        frontRight.driveMotor.stopMotor();
        backLeft.driveMotor.stopMotor();
        backRight.driveMotor.stopMotor();

        // Makes an X pattern with the swerve base
        // Set the modules to 45 degree angles
        frontLeft.setDesiredAngle(-45);
        frontRight.setDesiredAngle(45);
        backLeft.setDesiredAngle(45);
        backRight.setDesiredAngle(-45);
    }

    /** Updates the odometry. Should be called as frequently as possible to reduce error. */
    public void updateOdometry() {
        poseEstimator.update(
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
        poseEstimator.resetPosition(currentPosition, RobotContainer.navX.getRotation2d());
    }

    /** Adds a vision position measurement */
    public void visionPositionMeasurement(Pose2d visionRobotPose) {
        poseEstimator.addVisionMeasurement(visionRobotPose, Timer.getFPGATimestamp());
    }

    /**
     * Moves the robot to follow a trajectory
     *
     * @param desiredPosition The desired position of the robot
     * @param linearVelocity The linear velocity, in m/s, for the movement
     */
    public void trajectoryFollow(Pose2d desiredPosition, double linearVelocity) {

        // Calculate the velocities for the chassis
        ChassisSpeeds adjustedVelocities =
                driveController.calculate(
                        poseEstimator.getEstimatedPosition(),
                        desiredPosition,
                        linearVelocity,
                        desiredPosition.getRotation());

        // Set the modules to move at those velocities
        setModuleStates(adjustedVelocities);
    }

    /**
     * Gets the estimated X position of the drivetrain on the field
     *
     * @return Estimated X position (m)
     */
    public double getXPosition() {
        return poseEstimator.getEstimatedPosition().getX();
    }

    /**
     * Gets the estimated Y position of the drivetrain on the field
     *
     * @return Estimated Y position (m)
     */
    public double getYPosition() {
        return poseEstimator.getEstimatedPosition().getY();
    }

    /**
     * Gets the estimated angle of the drivetrain on the field
     *
     * @return Estimated angle (Rotation2d)
     */
    public Rotation2d getThetaPosition() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Gets the orientation of the robot on the field
     *
     * @return The orientation of the robot (Pose2d) (units in m)
     */
    public Pose2d getPose2D() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Check if the robot is at the reference point of the planned movement
     *
     * @return Is the movement finished?
     */
    public boolean finishedMovement() {
        return driveController.atReference();
    }

    /** Updates all of the steering parameters, such as PID loops and driver settings */
    public void updateParameters() {

        // Update the PID parameters with the new driver profile values
        Parameters.driveTrain.pid.FL_STEER_PID.setPeakOutput(
                Parameters.driver.currentProfile.maxModVelocity);
        Parameters.driveTrain.pid.FR_STEER_PID.setPeakOutput(
                Parameters.driver.currentProfile.maxModVelocity);
        Parameters.driveTrain.pid.BL_STEER_PID.setPeakOutput(
                Parameters.driver.currentProfile.maxModVelocity);
        Parameters.driveTrain.pid.BR_STEER_PID.setPeakOutput(
                Parameters.driver.currentProfile.maxModVelocity);

        // Set steering parameters
        frontLeft.setSteerMParams(
                Parameters.driveTrain.pid.FL_STEER_PID,
                Parameters.driver.currentProfile.steerIdleMode);
        frontRight.setSteerMParams(
                Parameters.driveTrain.pid.FR_STEER_PID,
                Parameters.driver.currentProfile.steerIdleMode);
        backLeft.setSteerMParams(
                Parameters.driveTrain.pid.BL_STEER_PID,
                Parameters.driver.currentProfile.steerIdleMode);
        backRight.setSteerMParams(
                Parameters.driveTrain.pid.BR_STEER_PID,
                Parameters.driver.currentProfile.steerIdleMode);

        // Set driving parameters
        frontLeft.setDriveMParams(
                Parameters.driveTrain.pid.FL_DRIVE_PID,
                Parameters.driver.currentProfile.driveIdleMode);
        frontRight.setDriveMParams(
                Parameters.driveTrain.pid.FR_DRIVE_PID,
                Parameters.driver.currentProfile.driveIdleMode);
        backLeft.setDriveMParams(
                Parameters.driveTrain.pid.BL_DRIVE_PID,
                Parameters.driver.currentProfile.driveIdleMode);
        backRight.setDriveMParams(
                Parameters.driveTrain.pid.BR_DRIVE_PID,
                Parameters.driver.currentProfile.driveIdleMode);
    }

    // Sets all of the modules to treat their current position as the zero position.
    public void zeroEncoders() {

        // Go through the CANCoders, setting each to zero
        frontLeft.setEncoderOffset(0);
        frontRight.setEncoderOffset(0);
        backLeft.setEncoderOffset(0);
        backRight.setEncoderOffset(0);
    }

    // Saves all of the parameters currently in the swerve modules
    public void saveParameters() {

        // Save module values
        frontLeft.saveParameters();
        frontRight.saveParameters();
        backLeft.saveParameters();
        backRight.saveParameters();

        // X Movement PID
        Preferences.setDouble("DRIVETRAIN_X_MOVE_PID_P", X_MOVE_PID.getP());
        Preferences.setDouble("DRIVETRAIN_X_MOVE_PID_I", X_MOVE_PID.getI());
        Preferences.setDouble("DRIVETRAIN_X_MOVE_PID_D", X_MOVE_PID.getD());

        // Y Movement PID
        Preferences.setDouble("DRIVETRAIN_Y_MOVE_PID_P", Y_MOVE_PID.getP());
        Preferences.setDouble("DRIVETRAIN_Y_MOVE_PID_I", Y_MOVE_PID.getI());
        Preferences.setDouble("DRIVETRAIN_Y_MOVE_PID_D", Y_MOVE_PID.getD());

        // Rotation PID (PID values)
        Preferences.setDouble("DRIVETRAIN_ROTATION_PID_P", ROTATION_PID.getP());
        Preferences.setDouble("DRIVETRAIN_ROTATION_PID_I", ROTATION_PID.getI());
        Preferences.setDouble("DRIVETRAIN_ROTATION_PID_D", ROTATION_PID.getD());

        // TODO: Add rotational PID constraints
    }

    /** Loads all of the currently saved parameters */
    public void loadParameters() {

        // Load module values
        frontLeft.loadParameters();
        frontRight.loadParameters();
        backLeft.loadParameters();
        backRight.loadParameters();

        // X Movement PID
        X_MOVE_PID.setP(Preferences.getDouble("DRIVETRAIN_X_MOVE_PID_P", X_MOVE_PID.getP()));
        X_MOVE_PID.setI(Preferences.getDouble("DRIVETRAIN_X_MOVE_PID_I", X_MOVE_PID.getI()));
        X_MOVE_PID.setD(Preferences.getDouble("DRIVETRAIN_X_MOVE_PID_D", X_MOVE_PID.getD()));

        // Y Movement PID
        Y_MOVE_PID.setP(Preferences.getDouble("DRIVETRAIN_Y_MOVE_PID_P", Y_MOVE_PID.getP()));
        Y_MOVE_PID.setI(Preferences.getDouble("DRIVETRAIN_Y_MOVE_PID_I", Y_MOVE_PID.getI()));
        Y_MOVE_PID.setD(Preferences.getDouble("DRIVETRAIN_Y_MOVE_PID_D", Y_MOVE_PID.getD()));

        // Rotation PID (PID values)
        ROTATION_PID.setP(Preferences.getDouble("DRIVETRAIN_ROTATION_PID_P", ROTATION_PID.getP()));
        ROTATION_PID.setI(Preferences.getDouble("DRIVETRAIN_ROTATION_PID_I", ROTATION_PID.getI()));
        ROTATION_PID.setD(Preferences.getDouble("DRIVETRAIN_ROTATION_PID_D", ROTATION_PID.getD()));

        // Rotation PID (Constraints)
        // TODO: Fix!
        /*
        double maxVelocity =
                Math.toRadians(
                        Preferences.getDouble(
                                "DRIVETRAIN_ROTATION_PID_MAX_VEL",
                                Math.toDegrees(ROTATION_CONSTRAINTS.maxVelocity)));
        double maxAcceleration =
                Math.toRadians(
                        Preferences.getDouble(
                                "DRIVETRAIN_ROTATION_PID_MAX_ACCEL",
                                Math.toDegrees(ROTATION_CONSTRAINTS.maxAcceleration)));

        // Create a new rotation PID object, then set it
        ROTATION_CONSTRAINTS = new Constraints(maxVelocity, maxAcceleration);
        ROTATION_PID.setConstraints(ROTATION_CONSTRAINTS);
        */

        // Publish the new tuning values
        publishTuningValues();
    }

    /** Loads all of the NetworkTable parameters */
    public void pullTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Pull module tuning values
            frontLeft.pullTuningValues();
            frontRight.pullTuningValues();
            backLeft.pullTuningValues();
            backRight.pullTuningValues();

            // X Movement PID
            X_MOVE_PID.setP(X_MOVE_PID_P_ENTRY.getDouble(X_MOVE_PID.getP()));
            X_MOVE_PID.setI(X_MOVE_PID_I_ENTRY.getDouble(X_MOVE_PID.getI()));
            X_MOVE_PID.setD(X_MOVE_PID_D_ENTRY.getDouble(X_MOVE_PID.getD()));

            // Y Movement PID
            Y_MOVE_PID.setP(Y_MOVE_PID_P_ENTRY.getDouble(Y_MOVE_PID.getP()));
            Y_MOVE_PID.setI(Y_MOVE_PID_I_ENTRY.getDouble(Y_MOVE_PID.getI()));
            Y_MOVE_PID.setD(Y_MOVE_PID_D_ENTRY.getDouble(Y_MOVE_PID.getD()));

            // Rotation PID (PID values)
            ROTATION_PID.setP(ROTATION_PID_P_ENTRY.getDouble(ROTATION_PID.getP()));
            ROTATION_PID.setI(ROTATION_PID_I_ENTRY.getDouble(ROTATION_PID.getI()));
            ROTATION_PID.setD(ROTATION_PID_D_ENTRY.getDouble(ROTATION_PID.getD()));

            // Rotation PID (Constraints)
            // TODO: Fix
            /*
            double maxVelocity =
                    Math.toRadians(
                            ROTATION_PID_MAX_VEL_ENTRY.getDouble(
                                    Math.toDegrees(ROTATION_CONSTRAINTS.maxVelocity)));
            double maxAcceleration =
                    Math.toRadians(
                            ROTATION_PID_MAX_ACCEL_ENTRY.getDouble(
                                    Math.toDegrees(ROTATION_CONSTRAINTS.maxAcceleration)));

            // Create a new rotation PID contraints object
            ROTATION_CONSTRAINTS = new Constraints(maxVelocity, maxAcceleration);
            ROTATION_PID.setConstraints(ROTATION_CONSTRAINTS);
            */
        }
    }

    /** Pushes all of the NetworkTable parameters */
    public void publishTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Publish module tuning values
            frontLeft.publishTuningValues();
            frontRight.publishTuningValues();
            backLeft.publishTuningValues();
            backRight.publishTuningValues();

            // X Movement PID
            X_MOVE_PID_P_ENTRY.setDouble(X_MOVE_PID.getP());
            X_MOVE_PID_I_ENTRY.setDouble(X_MOVE_PID.getI());
            X_MOVE_PID_D_ENTRY.setDouble(X_MOVE_PID.getD());

            // Y Movement PID
            Y_MOVE_PID_P_ENTRY.setDouble(Y_MOVE_PID.getP());
            Y_MOVE_PID_I_ENTRY.setDouble(Y_MOVE_PID.getI());
            Y_MOVE_PID_D_ENTRY.setDouble(Y_MOVE_PID.getD());

            // Rotation PID (PID values)
            ROTATION_PID_P_ENTRY.setDouble(ROTATION_PID.getP());
            ROTATION_PID_I_ENTRY.setDouble(ROTATION_PID.getI());
            ROTATION_PID_D_ENTRY.setDouble(ROTATION_PID.getD());

            // Rotation PID (Constraints)
            // TODO: Fix
            /*
            ROTATION_PID_MAX_VEL_ENTRY.setDouble(Math.toDegrees(ROTATION_CONSTRAINTS.maxVelocity));
            ROTATION_PID_MAX_ACCEL_ENTRY.setDouble(
                    Math.toDegrees(ROTATION_CONSTRAINTS.maxAcceleration));
            */
        }
    }

    /** Publish the performance data from each of the modules to the NetworkTable */
    public void publishPerformanceData() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Publish module velocity/angle
            frontLeft.publishPerformanceData();
            frontRight.publishPerformanceData();
            backLeft.publishPerformanceData();
            backRight.publishPerformanceData();

            // Publish the positional data of the robot
            X_POSITION_ENTRY.setDouble(getXPosition());
            Y_POSITION_ENTRY.setDouble(getYPosition());
            ROTATIONAL_POSITION_ENTRY.setDouble(getThetaPosition().getDegrees());
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
