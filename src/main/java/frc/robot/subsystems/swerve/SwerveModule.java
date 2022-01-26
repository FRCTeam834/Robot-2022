/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/8/20
 */
package frc.robot.subsystems.swerve;

// Imports
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;

public class SwerveModule {

    // Define all of the variables in the global scope

    // Motors
    public CANSparkMax steerMotor;
    public CANSparkMax driveMotor;
    private CachedPIDController steerMotorPID;
    private CachedPIDController driveMotorPID;
    public CANCoder steerCANCoder;
    private RelativeEncoder steerMotorEncoder;
    private RelativeEncoder driveMotorEncoder;

    // General info
    private double cancoderOffset = 0;
    private double angularOffset = 0;
    private String name;

    // PID control types
    CANSparkMax.ControlType steerMControlType;
    CANSparkMax.ControlType driveMControlType;

    // NetworkTable values
    private NetworkTableEntry steerPEntry;
    private NetworkTableEntry steerIEntry;
    private NetworkTableEntry steerIZoneEntry;
    private NetworkTableEntry steerDEntry;
    private NetworkTableEntry steerFFEntry;

    private NetworkTableEntry drivePEntry;
    private NetworkTableEntry driveIEntry;
    private NetworkTableEntry driveIZoneEntry;
    private NetworkTableEntry driveDEntry;
    private NetworkTableEntry driveFFEntry;

    private NetworkTableEntry currentVelocity;
    private NetworkTableEntry currentAngle;

    /**
     * Set up the module and address each of the motor controllers
     *
     * @param moduleName The name of the module (used on NetworkTables)
     * @param steerMID The CAN ID of the steer motor
     * @param driveMID The CAN ID of the drive motor
     * @param CANCoderID The CAN ID of the CANCoder angle sensor
     * @param steerPIDParams The PID parameters object for the steer motor
     * @param drivePIDParams The PID parameters object for the drive motor
     * @param reversedDrive If the drive motor should be reversed
     */
    public SwerveModule(
            String moduleName, int steerMID, int driveMID, int CANCoderID, boolean reversedDrive) {

        // Set the name
        this.name = moduleName;

        // CANCoder
        this.steerCANCoder = new CANCoder(CANCoderID);
        this.steerCANCoder.setPositionToAbsolute();
        this.steerCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.steerCANCoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition);

        // Steering motor
        this.steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
        this.steerMotor.restoreFactoryDefaults();
        this.steerMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);
        this.steerMotor.setIdleMode(IdleMode.kBrake);
        this.steerMotor.setSmartCurrentLimit(Parameters.driveTrain.maximums.MAX_STEER_CURRENT);
        this.steerMotor.setInverted(false);
        this.steerMotor.setSmartCurrentLimit(20);

        // Steer motor encoder (position is converted from rotations to degrees)
        // (For the conversion factor) First we multiply by 360 to convert rotations to degrees,
        // then divide by the steer gear ratio because the motor must move that many times for a
        // full
        // module rotation
        // For the velocity, we can use the same conversion factor and divide by 60 to convert RPM
        // to deg/s
        this.steerMotorEncoder = this.steerMotor.getEncoder();
        this.steerMotorEncoder.setPositionConversionFactor(
                360.0 / Parameters.driveTrain.ratios.STEER_GEAR_RATIO);
        this.steerMotorEncoder.setVelocityConversionFactor(
                360.0 / (Parameters.driveTrain.ratios.STEER_GEAR_RATIO * 60));
        this.steerMotorEncoder.setPosition(this.getAngle());

        // Steering PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we can only send a set amount across at once.
        this.steerMotorPID = new CachedPIDController(this.steerMotor);
        this.steerMotorPID.setP(Parameters.driveTrain.pid.steer.DEFAULT_P);
        this.steerMotorPID.setI(Parameters.driveTrain.pid.steer.DEFAULT_I);
        this.steerMotorPID.setIZone(Parameters.driveTrain.pid.steer.DEFAULT_I_ZONE);
        this.steerMotorPID.setD(Parameters.driveTrain.pid.steer.DEFAULT_D);
        this.steerMotorPID.setFF(Parameters.driveTrain.pid.steer.DEFAULT_FF);
        this.steerMotorPID.setOutputRange(-1, 1);

        // Set the angular velocity and acceleration values (if smart motion is being used)
        if (Parameters.driveTrain.pid.steer.DEFAULT_CONTROL_TYPE.equals(ControlType.kSmartMotion)) {
            this.steerMotorPID.setSmartMotionMaxAccel(Parameters.driveTrain.maximums.MAX_ACCEL);
            this.steerMotorPID.setSmartMotionMaxVelocity(
                    Parameters.driveTrain.maximums.MAX_VELOCITY);
            this.steerMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal);
        }

        // Save the control type for the steering motor
        this.steerMControlType = Parameters.driveTrain.pid.steer.DEFAULT_CONTROL_TYPE;

        // Drive motor
        this.driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
        this.driveMotor.restoreFactoryDefaults();
        this.driveMotor.enableVoltageCompensation(Parameters.general.nominalVoltage);
        this.driveMotor.setSmartCurrentLimit(Parameters.driveTrain.maximums.MAX_DRIVE_CURRENT);
        this.driveMotor.setIdleMode(Parameters.driver.driveIdleMode);
        this.driveMotor.setSmartCurrentLimit(30);

        // Reverse the motor direction if specified
        this.driveMotor.setInverted(reversedDrive);

        // Drive motor encoder
        // First we need to multiply by min/sec (1/60) to get to rotations/s
        // Then we divide by the drive gear ratio, converting motor rotations/s to wheel rotations/s
        // Finally, we multiply by Pi * d, which is the circumference of the wheel, converting it to
        // wheel m/s
        // It's similar with position, we just don't need to divide by 60. Converts rotations to
        // meters
        this.driveMotorEncoder = this.driveMotor.getEncoder();
        this.driveMotorEncoder.setVelocityConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / (60.0 * Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO));
        this.driveMotorEncoder.setPositionConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO);

        // Drive motor PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we can only send a set amount across at once.
        this.driveMotorPID = new CachedPIDController(this.driveMotor);
        this.driveMotorPID.setP(Parameters.driveTrain.pid.drive.DEFAULT_P);
        this.driveMotorPID.setI(Parameters.driveTrain.pid.drive.DEFAULT_I);
        this.driveMotorPID.setIZone(Parameters.driveTrain.pid.drive.DEFAULT_I_ZONE);
        this.driveMotorPID.setD(Parameters.driveTrain.pid.drive.DEFAULT_D);
        this.driveMotorPID.setFF(Parameters.driveTrain.pid.drive.DEFAULT_FF);
        this.driveMotorPID.setOutputRange(-1, 1);

        // Save the control type for the drive motor
        this.driveMControlType = Parameters.driveTrain.pid.drive.DEFAULT_CONTROL_TYPE;

        // Burn the flash parameters to the Sparks (prevents loss of parameters after brownouts)
        this.steerMotor.burnFlash();
        this.driveMotor.burnFlash();

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Set up the module's table on NetworkTables
            NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
            NetworkTable moduleTable = swerveTable.getSubTable(name + "_MODULE");

            // Put all of the module's current values on NetworkTables
            // Steer PID
            this.steerPEntry = moduleTable.getEntry("STEER_P");
            this.steerIEntry = moduleTable.getEntry("STEER_I");
            this.steerIZoneEntry = moduleTable.getEntry("STEER_I_ZONE");
            this.steerDEntry = moduleTable.getEntry("STEER_D");
            this.steerFFEntry = moduleTable.getEntry("STEER_FF");

            // Drive PID
            this.drivePEntry = moduleTable.getEntry("DRIVE_P");
            this.driveIEntry = moduleTable.getEntry("DRIVE_I");
            this.driveIZoneEntry = moduleTable.getEntry("DRIVE_I_ZONE");
            this.driveDEntry = moduleTable.getEntry("DRIVE_D");
            this.driveFFEntry = moduleTable.getEntry("DRIVE_FF");

            // Performance data
            this.currentVelocity = moduleTable.getEntry("CURRENT_VELOCITY");
            this.currentAngle = moduleTable.getEntry("CURRENT_ANGLE");
        }
    }

    /**
     * Moves the wheel to the target angle, complete with optimizations
     *
     * @param targetAngle The angle to move the module to
     */
    public void setDesiredAngle(double targetAngle) {

        // Motor angle optimization code (makes sure that the motor doesn't go all the way
        // around)
        while (Math.abs(this.getAdjSteerMotorAng() - targetAngle) >= 90) {

            // Calculate the angular deviation
            double angularDev = this.getAdjSteerMotorAng() - targetAngle;

            // Full rotation optimizations
            if (angularDev >= 180) {
                this.angularOffset += 360;
            } else if (angularDev <= -180) {
                this.angularOffset -= 360;
            }

            // Half rotation optimizations (full are prioritized first)
            else if (angularDev >= 90) {
                this.angularOffset -= 180;
                this.driveMotor.setInverted(!this.driveMotor.getInverted());
            } else if (angularDev <= -90) {
                this.angularOffset -= 180;
                this.driveMotor.setInverted(!this.driveMotor.getInverted());
            }
        }

        // Set the PID reference
        this.steerMotorPID.setReference(targetAngle + angularOffset, steerMControlType);

        // Print out info (for debugging)
        if (Parameters.debug) {
            this.printDebugString(targetAngle);
        }
    }

    /**
     * Checks if the module is at it's desired angle
     *
     * @return Has the module reached it's desired angle?
     */
    public boolean isAtDesiredAngle(double desiredAngle) {

        // Get the current angle of the module
        double currentAngle = this.getAngle();

        // Return if the module has reached the desired angle
        return (currentAngle < (desiredAngle + Parameters.driveTrain.angleTolerance)
                && (currentAngle > (desiredAngle - Parameters.driveTrain.angleTolerance)));
    }

    // Sets the power of the drive motor
    public void setRawDrivePower(double percentage) {
        this.driveMotor.set(percentage);
    }

    // Set the desired velocity in m/s
    public void setDesiredVelocity(double targetVelocity) {

        // Calculate the output of the drive
        this.driveMotorPID.setReference(targetVelocity, this.driveMControlType);

        // Print out debug info if needed
        if (Parameters.debug) {
            System.out.println("D_SPD: " + targetVelocity + " | A_SPD: " + getVelocity());
        }
    }

    // Sets the desired velocity in m/s (proportional to the error of the angle)
    public boolean setDesiredVelocity(double targetVelocity, double targetAngle) {

        // Declare the percent error storage
        double percentFactor = 1;

        // Check to make sure that the value is within 90 degrees (no movement until within 90)
        if (Math.abs((targetAngle + this.angularOffset) - this.getActualSteerMotorAngle()) <= 90) {

            // Compute the error factor (based on how close the actual angle is to the desired)
            percentFactor =
                    1
                            - Math.abs(
                                    ((targetAngle + this.angularOffset)
                                                    - this.getActualSteerMotorAngle())
                                            / 90);

            // Print the percent error if debugging is enabled
            if (Parameters.debug) {
                System.out.println("% E: " + percentFactor);
            }

            // Set the adjusted velocity
            this.setDesiredVelocity(targetVelocity * percentFactor);
        }

        // Return if we have reached our desired velocity (should always return correctly,
        // regardless of enable state)
        return this.isAtDesiredVelocity(targetVelocity * percentFactor);
    }

    // Checks if a module's velocity is within tolerance
    public boolean isAtDesiredVelocity(double desiredVelocity) {

        // Get the current velocity of the drive motor
        double currentVelocity = this.getVelocity();

        // Return if the velocity is within tolerance
        return ((currentVelocity < (desiredVelocity + Parameters.driveTrain.velocityTolerance))
                && (currentVelocity > (desiredVelocity - Parameters.driveTrain.velocityTolerance)));
    }

    // Sets the desired state of the module
    public void setDesiredState(SwerveModuleState setState) {

        // Set module to the right angles and velocities
        this.setDesiredAngle(setState.angle.getDegrees());
        this.setDesiredVelocity(setState.speedMetersPerSecond, setState.angle.getDegrees());
    }

    // Gets the state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getVelocity(), Rotation2d.fromDegrees(this.getAngle()));
    }

    // Gets the position of the encoder (in deg)
    public double getAngle() {
        return this.steerCANCoder.getAbsolutePosition();
    }

    // Gets the adjusted steer motor's angle
    public double getAdjSteerMotorAng() {
        return (this.getActualSteerMotorAngle() - angularOffset);
    }

    // Gets the actual steer motor's angle
    public double getActualSteerMotorAngle() {
        return this.steerMotorEncoder.getPosition();
    }

    // Returns the velocity of the module (from the wheel)
    public double getVelocity() {
        return this.driveMotorEncoder.getVelocity();
    }

    // Sets the position of the encoder
    public void setEncoderOffset(double correctPosition) {

        // Set the cancoder offset variable
        this.cancoderOffset =
                correctPosition - (this.getAngle() - this.steerCANCoder.configGetMagnetOffset());

        // Set the offset on the encoder
        this.steerCANCoder.configMagnetOffset(this.cancoderOffset);

        // Set the encoder's position to zero
        // The getAngle reference should be changed now, so we need to re-request it
        this.steerMotorEncoder.setPosition(this.getAngle());
    }

    // Stop both of the motors
    public void stopMotors() {

        // Shut off all of the motors
        this.steerMotor.stopMotor();
        this.driveMotor.stopMotor();
    }

    // Saves all parameters relevant to the swerve module
    public void saveParameters() {
        // Saves the configured configuration in the present tense

        // Steer PID
        Preferences.setDouble(this.name + "_STEER_P", this.steerMotorPID.getP());
        Preferences.setDouble(this.name + "_STEER_I", this.steerMotorPID.getI());
        Preferences.setDouble(this.name + "_STEER_I_ZONE", this.steerMotorPID.getIZone());
        Preferences.setDouble(this.name + "_STEER_D", this.steerMotorPID.getD());
        Preferences.setDouble(this.name + "_STEER_FF", this.steerMotorPID.getFF());

        // Drive PID
        Preferences.setDouble(this.name + "_DRIVE_P", this.driveMotorPID.getP());
        Preferences.setDouble(this.name + "_DRIVE_I", this.driveMotorPID.getI());
        Preferences.setDouble(this.name + "_DRIVE_I_ZONE", this.driveMotorPID.getIZone());
        Preferences.setDouble(this.name + "_DRIVE_D", this.driveMotorPID.getD());
        Preferences.setDouble(this.name + "_DRIVE_FF", this.driveMotorPID.getFF());

        // Encoder offset
        Preferences.setDouble(this.name + "_ENCODER_OFFSET", this.cancoderOffset);
    }

    // Loads all of the parameters from the Rio's saved data
    public void loadParameters() {

        // Steer PID
        this.steerMotorPID.setP(
                Preferences.getDouble(this.name + "_STEER_P", this.steerMotorPID.getP()));
        this.steerMotorPID.setI(
                Preferences.getDouble(this.name + "_STEER_I", this.steerMotorPID.getI()));
        this.steerMotorPID.setIZone(
                Preferences.getDouble(this.name + "_STEER_I_ZONE", this.steerMotorPID.getIZone()));
        this.steerMotorPID.setD(
                Preferences.getDouble(this.name + "_STEER_D", this.steerMotorPID.getD()));
        this.steerMotorPID.setFF(
                Preferences.getDouble(this.name + "_STEER_FF", this.steerMotorPID.getFF()));

        // Drive PID
        this.driveMotorPID.setP(
                Preferences.getDouble(this.name + "_DRIVE_P", this.driveMotorPID.getP()));
        this.driveMotorPID.setI(
                Preferences.getDouble(this.name + "_DRIVE_I", this.driveMotorPID.getI()));
        this.driveMotorPID.setIZone(
                Preferences.getDouble(this.name + "_DRIVE_I_ZONE", this.driveMotorPID.getIZone()));
        this.driveMotorPID.setD(
                Preferences.getDouble(this.name + "_DRIVE_D", this.driveMotorPID.getD()));
        this.driveMotorPID.setFF(
                Preferences.getDouble(this.name + "_DRIVE_FF", this.driveMotorPID.getFF()));

        // Encoder offset
        this.steerCANCoder.configMagnetOffset(
                Preferences.getDouble(this.name + "_ENCODER_OFFSET", this.cancoderOffset));
        this.steerMotorEncoder.setPosition(this.getAngle());

        // Push the new values to the table
        this.publishTuningValues();
    }

    // Resets all of the parameters to their default values (as per Parameters.java upon compile)
    public void defaultParameters() {

        // Steer PID
        this.steerMotorPID.setP(Parameters.driveTrain.pid.steer.DEFAULT_P);
        this.steerMotorPID.setI(Parameters.driveTrain.pid.steer.DEFAULT_I);
        this.steerMotorPID.setIZone(Parameters.driveTrain.pid.steer.DEFAULT_I_ZONE);
        this.steerMotorPID.setD(Parameters.driveTrain.pid.steer.DEFAULT_D);
        this.steerMotorPID.setFF(Parameters.driveTrain.pid.steer.DEFAULT_FF);

        // Drive PID
        this.driveMotorPID.setP(Parameters.driveTrain.pid.drive.DEFAULT_P);
        this.driveMotorPID.setI(Parameters.driveTrain.pid.drive.DEFAULT_I);
        this.driveMotorPID.setIZone(Parameters.driveTrain.pid.drive.DEFAULT_I_ZONE);
        this.driveMotorPID.setD(Parameters.driveTrain.pid.drive.DEFAULT_D);
        this.driveMotorPID.setFF(Parameters.driveTrain.pid.drive.DEFAULT_FF);

        // Push the new values to the table
        this.publishTuningValues();
    }

    // Push the values to NetworkTables
    public void publishTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Steer PIDs
            this.steerPEntry.setDouble(this.steerMotorPID.getP());
            this.steerIEntry.setDouble(this.steerMotorPID.getI());
            this.steerIZoneEntry.setDouble(this.steerMotorPID.getIZone());
            this.steerDEntry.setDouble(this.steerMotorPID.getD());
            this.steerFFEntry.setDouble(this.steerMotorPID.getFF());

            // Drive PIDs
            this.drivePEntry.setDouble(this.driveMotorPID.getP());
            this.driveIEntry.setDouble(this.driveMotorPID.getI());
            this.driveIZoneEntry.setDouble(this.driveMotorPID.getIZone());
            this.driveDEntry.setDouble(this.driveMotorPID.getD());
            this.driveFFEntry.setDouble(this.driveMotorPID.getFF());
        }
    }

    // Get the values from NetworkTables
    public void pullTuningValues() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {

            // Steer PIDs
            this.steerMotorPID.setP(this.steerPEntry.getDouble(this.steerMotorPID.getP()));
            this.steerMotorPID.setI(this.steerIEntry.getDouble(this.steerMotorPID.getI()));
            this.steerMotorPID.setIZone(
                    this.steerIZoneEntry.getDouble(this.steerMotorPID.getIZone()));
            this.steerMotorPID.setD(this.steerDEntry.getDouble(this.steerMotorPID.getD()));
            this.steerMotorPID.setFF(this.steerFFEntry.getDouble(this.steerMotorPID.getFF()));

            // Drive PIDs
            this.driveMotorPID.setP(this.drivePEntry.getDouble(this.driveMotorPID.getP()));
            this.driveMotorPID.setI(this.driveIEntry.getDouble(this.driveMotorPID.getI()));
            this.driveMotorPID.setIZone(
                    this.driveIZoneEntry.getDouble(this.driveMotorPID.getIZone()));
            this.driveMotorPID.setD(this.driveDEntry.getDouble(this.driveMotorPID.getD()));
            this.driveMotorPID.setFF(this.driveFFEntry.getDouble(this.driveMotorPID.getFF()));
        }
    }

    // Pushes the performance data to the NetworkTable
    public void publishPerformanceData() {

        // Don't mess with NetworkTables unless we have to
        if (Parameters.networkTables) {
            this.currentVelocity.setDouble(this.getVelocity());
            this.currentAngle.setDouble(this.getAngle());
        }
    }

    // Print out a debug string
    public void printDebugString(double targetAngle) {
        System.out.println(
                this.name
                        + ": TAR_A: "
                        + Math.round(targetAngle)
                        + " ACT_A: "
                        + Math.round(this.getAngle())
                        + " ADJ_A: "
                        + Math.round(this.getAdjSteerMotorAng())
                        + " STR_A: "
                        + Math.round(this.getActualSteerMotorAngle())
                        + " OFF_A: "
                        + Math.round(this.angularOffset));
    }
}
