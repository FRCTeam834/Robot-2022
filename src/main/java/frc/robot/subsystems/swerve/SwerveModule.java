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
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.Parameters.driveTrain.pid;
import frc.robot.utilityClasses.CachedPIDController;
import frc.robot.utilityClasses.PIDParams;

public class SwerveModule extends SubsystemBase {

    // Define all of the variables in the global scope

    // Motors
    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;
    private CachedPIDController steerMotorPID;
    private CachedPIDController driveMotorPID;
    private CANCoder steerCANCoder;
    private RelativeEncoder steerMotorEncoder;
    private RelativeEncoder driveMotorEncoder;

    // General info
    private double cancoderOffset = 0;
    private double angularOffset = 0;
    private double desiredAngle = 0; // in deg
    private double desiredVelocity = 0; // in m/s
    private String name;

    // Feed forward for the motor
    private SimpleMotorFeedforward driveFF;

    // If the module is reversed (for angle optimization)
    private boolean isDriveReversed;

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
        name = moduleName;

        // CANCoder
        steerCANCoder = new CANCoder(CANCoderID);
        steerCANCoder.setPositionToAbsolute();
        steerCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        steerCANCoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition);

        // Steering motor
        steerMotor = new CANSparkMax(steerMID, CANSparkMax.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.enableVoltageCompensation(12);
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setSmartCurrentLimit(20);
        steerMotor.setInverted(false);

        // Steer motor encoder (position is converted from rotations to degrees)
        // (For the conversion factor) First we multiply by 360 to convert rotations to degrees,
        // then divide by the steer gear ratio because the motor must move that many times for a
        // full module rotation. For the velocity, we can use the same conversion factor and divide
        // by 60 to convert RPM to deg/s
        steerMotorEncoder = steerMotor.getEncoder();
        steerMotorEncoder.setPositionConversionFactor(
                360.0 / Parameters.driveTrain.ratios.STEER_GEAR_RATIO);
        steerMotorEncoder.setVelocityConversionFactor(
                360.0 / (Parameters.driveTrain.ratios.STEER_GEAR_RATIO * 60));
        steerMotorEncoder.setPosition(getTrueAngle());

        // Steering PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we can only send a set amount across at once.
        steerMotorPID = new CachedPIDController(steerMotor);
        steerMotorPID.setP(pid.steer.kP.get());
        steerMotorPID.setD(pid.steer.kD.get());
        steerMotorPID.setOutputRange(-1, 1);

        // Set the angular velocity and acceleration values (if smart motion is being used)
        if (pid.steer.CONTROL_TYPE.equals(ControlType.kSmartMotion)) {
            steerMotorPID.setSmartMotionMaxAccel(Parameters.driveTrain.maximums.MAX_STEER_ACCEL);
            steerMotorPID.setSmartMotionMaxVelocity(
                    Parameters.driveTrain.maximums.MAX_STEER_VELOCITY);
            steerMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal);
        }

        // Drive motor
        driveMotor = new CANSparkMax(driveMID, CANSparkMax.MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.enableVoltageCompensation(12);
        driveMotor.setSmartCurrentLimit(Parameters.driveTrain.maximums.MAX_DRIVE_CURRENT);
        driveMotor.setIdleMode(IdleMode.kBrake);

        // Reverse the motor direction if specified
        driveMotor.setInverted(reversedDrive);

        // Drive motor encoder
        // First we need to multiply by min/sec (1/60) to get to rotations/s
        // Then we divide by the drive gear ratio, converting motor rotations/s to wheel rotations/s
        // Finally, we multiply by Pi * d, which is the circumference of the wheel, converting it to
        // wheel m/s
        // It's similar with position, we just don't need to divide by 60. Converts rotations to
        // meters
        driveMotorEncoder = driveMotor.getEncoder();
        driveMotorEncoder.setPositionConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / (Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO));
        driveMotorEncoder.setVelocityConversionFactor(
                (Math.PI * Parameters.driveTrain.dimensions.MODULE_WHEEL_DIA_M)
                        / (60.0 * Parameters.driveTrain.ratios.DRIVE_GEAR_RATIO));

        // Drive motor PID controller (from motor)
        // Note that we use a "cached" controller.
        // This version of the PID controller checks if the desired setpoint is already set.
        // This reduces the load on the CAN bus, as we only have to send a set amount across at
        // once.
        driveMotorPID = new CachedPIDController(driveMotor);
        driveMotorPID.setP(pid.drive.kP.get());
        driveMotorPID.setD(pid.drive.kD.get());
        driveMotorPID.setOutputRange(-1, 1);

        // Burn the flash parameters to the Sparks (prevents loss of parameters after brownouts)
        if (Parameters.flashControllers) {
            steerMotor.burnFlash();
            driveMotor.burnFlash();
        }

        // Set up the feed forward
        driveFF =
                new SimpleMotorFeedforward(
                        Parameters.driveTrain.pid.drive.kFFS, Parameters.driveTrain.pid.drive.kFFV);

        // Set the CAN frame update rate
        // ! THIS IS EXTREMELY DANGEROUS, DO NOT TOUCH UNLESS YOU KNOW WHAT YOU ARE DOING!!!
        // ! THIS COULD CAUSE MOTORS TO STOP RESPONDING OR OTHERWISE MALFUNCTION!!!
        // No need to burn the flash, the update rates are not saved

        // Status 0 - faults, applied output (percentage), following
        // Status 1 - velocity, temperature, voltage, current
        // Status 2 - position
        // Status 3 - analog sensor data (never used)
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        steerMotor.setPeriodicFramePeriod(
                PeriodicFrame.kStatus1, 100); // We never use velocity, temp, or current
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);

        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000); // We never use position
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);

        // Seed if the motor should be reversed
        isDriveReversed = reversedDrive;

        // Load the offset of the encoder
        loadEncoderOffset();
    }

    /**
     * Sets the steer motor parameters
     *
     * @param pidParams The PID parameters
     * @param idleMode The idle mode of the motor
     */
    public void setSteerMParams(PIDParams pidParams, IdleMode idleMode) {

        // PID parameters
        steerMotorPID.setP(pidParams.kP);
        steerMotorPID.setI(pidParams.kI);
        steerMotorPID.setD(pidParams.kD);

        // Idle mode of the motor
        steerMotor.setIdleMode(idleMode);

        // Save the parameters (prevents loss of parameters after brownouts)
        steerMotor.burnFlash();
    }

    /**
     * Sets the drive motor parameters
     *
     * @param pidParams The PID parameters
     * @param idleMode The idle mode of the motor
     */
    public void setDriveMParams(PIDParams pidParams, IdleMode idleMode) {

        // PIDF parameters
        driveMotorPID.setP(pidParams.kP);
        driveMotorPID.setI(pidParams.kI);
        driveMotorPID.setD(pidParams.kD);

        // Idle mode of the motor
        driveMotor.setIdleMode(idleMode);

        // Save the parameters (prevents loss of parameters after brownouts)
        driveMotor.burnFlash();
    }

    /**
     * Gets the steering motor object for the selected module
     *
     * @return The steering motor object
     */
    public CANSparkMax getSteerMotor() {
        return steerMotor;
    }

    /**
     * Gets the drive motor object for the selected module
     *
     * @return The drive motor object
     */
    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    /**
     * Gets the CANCoder object for the selected module
     *
     * @return The CANCoder object
     */
    public CANCoder getCANCoder() {
        return steerCANCoder;
    }

    /**
     * Moves the wheel to the target angle, complete with optimizations
     *
     * @param targetAngle The angle to move the module to
     */
    public void setDesiredAngle(double targetAngle) {

        // Motor angle optimization code (makes sure that the motor doesn't go all the way
        // around)
        // Get the angle once, reducing the number of CAN bus readings we need to do
        double actualMotorAngle = getActSteerMotorAngle();

        // Keep optimizing until the angular deviation is below 90. We should never have to turn
        // past 90 degrees
        while (Math.abs((actualMotorAngle - angularOffset) - targetAngle) >= 90) {

            // Calculate the angular deviation
            double angularDev = (actualMotorAngle - angularOffset) - targetAngle;

            // Full rotation optimizations
            if (angularDev >= 180) {
                angularOffset += 360;
            } else if (angularDev <= -180) {
                angularOffset -= 360;
            }

            // Half rotation optimizations (full are prioritized first)
            else if (angularDev >= 90) {
                angularOffset += 180;
                invertDriveMotor();
            } else if (angularDev <= -90) {
                angularOffset -= 180;
                invertDriveMotor();
            }
        }

        // Calculate the optimal angle for the motor (needs to be corrected as it thinks that
        // the position is 0 at it's startup location)
        desiredAngle = targetAngle + angularOffset;

        // Set the PID reference
        steerMotorPID.setReference(desiredAngle, Parameters.driveTrain.pid.steer.CONTROL_TYPE);

        // Print out info (for debugging)
        if (Parameters.debug) {
            printDebugString(targetAngle);
        }
    }

    /** Inverts the direction of the drive motor */
    private void invertDriveMotor() {
        isDriveReversed = !isDriveReversed;
        driveMotor.setInverted(isDriveReversed);
    }

    /**
     * Checks if the module is at it's desired angle
     *
     * @return Has the module reached it's desired angle?
     */
    public boolean isAtDesiredAngle(double desiredAngle) {

        // Get the current angle of the module
        double currentAngle = getAdjAngle();

        // Return if the module has reached the desired angle
        return (currentAngle < (desiredAngle + Parameters.driveTrain.angleTolerance)
                && (currentAngle > (desiredAngle - Parameters.driveTrain.angleTolerance)));
    }

    // Set the desired velocity in m/s
    public void setDesiredVelocity(double targetVelocity) {

        // Calculate the output of the drive
        driveMotorPID.setReference(
                targetVelocity,
                Parameters.driveTrain.pid.drive.CONTROL_TYPE,
                driveFF.calculate(targetVelocity),
                ArbFFUnits.kVoltage);

        // Print out debug info if needed
        if (Parameters.debug) {
            System.out.println(
                    String.format("D_SPD: %.3f | A_SPD: %.3f", targetVelocity, getVelocity()));
        }

        // Save the desired velocity
        desiredVelocity = targetVelocity;
    }

    public void setDesiredVelocityOpenLoop(double speed) {
        driveMotor.set(speed);
    }

    // Sets the desired velocity in m/s (proportional to the error of the angle)
    public boolean setDesiredVelocity(double targetVelocity, double targetAngle) {

        // Compute the deviation from the setpoint
        double angularDev = Math.abs((targetAngle + angularOffset) - getActSteerMotorAngle());

        // Check to make sure that the value is within 90 degrees (no movement until within 90)
        if (angularDev <= 90) {

            // Compute the error factor (based on how close the actual angle is to the desired)
            double percentError = 1 - (angularDev / 90);

            // Print the percent error if debugging is enabled
            if (Parameters.debug) {
                System.out.println(String.format("E: %.2f", percentError));
            }

            // Set the adjusted velocity
            setDesiredVelocity(targetVelocity * percentError);
        }

        // Return if we have reached our desired velocity
        return isAtDesiredVelocity(targetVelocity);
    }

    // Checks if a module's velocity is within tolerance
    public boolean isAtDesiredVelocity(double desiredVelocity) {

        // Get the current velocity of the drive motor
        double currentVelocity = getVelocity();

        // Return if the velocity is within tolerance
        return ((currentVelocity < (desiredVelocity + Parameters.driveTrain.velocityTolerance))
                && (currentVelocity > (desiredVelocity - Parameters.driveTrain.velocityTolerance)));
    }

    // Sets the desired state of the module
    public void setDesiredState(SwerveModuleState setState, boolean openLoopDrive) {

        // Set module to the right angles and velocities

        setDesiredAngle(setState.angle.getDegrees());
        if (!openLoopDrive) {
            setDesiredVelocity(setState.speedMetersPerSecond, setState.angle.getDegrees());
        } else {
            setDesiredVelocityOpenLoop(
                    setState.speedMetersPerSecond
                            / Parameters.driveTrain.maximums.MAX_TRANS_VELOCITY);
        }
    }

    // Gets the state of the module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAdjAngle()));
    }

    // Gets the position of the encoder (in deg)
    public double getTrueAngle() {
        return steerCANCoder.getAbsolutePosition();
    }

    // Gets the position of the encoder (flipped with the drive motor to account for angle
    // optimizations)
    public double getAdjAngle() {
        return (steerCANCoder.getAbsolutePosition() - angularOffset);
    }

    // Gets the adjusted steer motor's angle
    public double getAdjSteerMotorAng() {
        return (getActSteerMotorAngle() - angularOffset);
    }

    // Gets the actual steer motor's angle
    public double getActSteerMotorAngle() {
        return steerMotorEncoder.getPosition();
    }

    // Returns the velocity of the module (from the wheel)
    public double getVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    // Sets the position of the encoder
    public void setEncoderOffset(double correctPosition) {

        // Set the CANCoder offset variable
        cancoderOffset = correctPosition - (getTrueAngle() - steerCANCoder.configGetMagnetOffset());

        // Set the offset on the encoder
        steerCANCoder.configMagnetOffset(cancoderOffset);

        // Set the encoder's position to zero
        // The getAngle reference should be changed now, so we need to re-request it
        reloadSteerAngle();

        if (Parameters.debug) {
            System.out.println(name + "_OFFSET: " + getCANCoder().getAbsolutePosition());
        }
    }

    public void setRawEncoderOffset(double offset) {
        steerCANCoder.configMagnetOffset(offset);
        reloadSteerAngle();
    }

    // Saves the module's encoder offset
    public void saveEncoderOffset() {

        // Encoder offset
        Preferences.setDouble(name + "_ENCODER_OFFSET", cancoderOffset);
    }

    // Loads the module's encoder offset
    public void loadEncoderOffset() {

        // Set the encoder offset
        steerCANCoder.configMagnetOffset(
                Preferences.getDouble(name + "_ENCODER_OFFSET", cancoderOffset));

        // Reload the steer angle
        reloadSteerAngle();
    }

    /**
     * Gets the angle of the swerve module from the CANCoder, then sets that the steer motor is at
     * that point
     */
    public void reloadSteerAngle() {
        steerMotorEncoder.setPosition(getTrueAngle());
        // driveMotor.setInverted(isReversed);
    }

    // Stop both of the motors
    public void stop() {

        // Shut off all of the motors
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    // Print out a debug string
    public void printDebugString(double targetAngle) {
        System.out.println(
                String.format(
                        "%s | TAR_A: %.2f | ACT_A: %.2f | ADJ_A: %.2f | MOT_A: %.2f | OFF: %.2f",
                        name,
                        targetAngle,
                        getAdjAngle(),
                        getAdjSteerMotorAng(),
                        getActSteerMotorAngle(),
                        angularOffset));
    }

    @Override
    public void periodic() {
        if (Parameters.tuningMode) {
            setDriveMParams(
                    new PIDParams(
                            pid.drive.kP.get(),
                            0,
                            pid.drive.kD.get(),
                            0,
                            pid.drive.kMAX_OUTPUT,
                            pid.drive.CONTROL_TYPE),
                    IdleMode.kBrake);
            setSteerMParams(
                    new PIDParams(
                            pid.steer.kP.get(),
                            0,
                            pid.steer.kD.get(),
                            0,
                            pid.steer.kMAX_OUTPUT,
                            pid.steer.CONTROL_TYPE),
                    IdleMode.kBrake);
        }
    }
}
