package frc.robot.subsystems.climber;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.CachedPIDController;
import frc.robot.utilityClasses.TuneableNumber;

public class TelescopingTube extends SubsystemBase {

    // Motor object
    CANSparkMax spoolMotor;

    // Motor encoder
    RelativeEncoder spoolMotorEncoder;

    // PID controller
    CachedPIDController pidController;

    // The motor's control type
    ControlType controlType;

    // Homing limit switch
    DigitalInput limitSwitch;

    // Variable to store if the spool has been homed yet
    boolean homed = false;

    // Tunable numbers
    TuneableNumber kP;
    TuneableNumber kD;

    // The min and max distances
    double minDistance;
    double maxDistance;

    // The tolerance for positioning
    double positionTolerance;

    public TelescopingTube(
            String name,
            int ID,
            int LSPort,
            double circumference,
            double gearboxRatio,
            double kP,
            double kD,
            double maxDuty,
            ControlType controlType,
            double minDist,
            double maxDist,
            double posTolerance) {

        // Get the table for tubes
        NetworkTable tubeTable = NetworkTableInstance.getDefault().getTable("TeleTubies");

        // Set up the tunable numbers
        this.kP = new TuneableNumber(tubeTable, name, kP);
        this.kD = new TuneableNumber(tubeTable, name, kD);

        // Initialize the spool motor
        spoolMotor = new CANSparkMax(ID, MotorType.kBrushless);
        spoolMotor.restoreFactoryDefaults();
        spoolMotor.enableVoltageCompensation(12);
        spoolMotor.setIdleMode(IdleMode.kBrake);
        spoolMotor.setSmartCurrentLimit(Parameters.climber.TUBE_CURRENT_LIMIT);
        spoolMotor.setInverted(false);

        // Set up the encoder of the spool motor
        spoolMotorEncoder = spoolMotor.getEncoder();
        spoolMotorEncoder.setPositionConversionFactor(circumference / gearboxRatio);
        spoolMotorEncoder.setVelocityConversionFactor(circumference / (gearboxRatio * 60));

        // Set up the PID controller
        pidController = new CachedPIDController(spoolMotor);
        pidController.setOutputRange(-maxDuty, maxDuty);
        pidController.setP(this.kP.get());
        pidController.setD(this.kD.get());

        // Set the control type
        this.controlType = controlType;

        // Set up the limit switch
        limitSwitch = new DigitalInput(LSPort);

        // Set the min and max distances
        minDistance = minDist;
        maxDistance = maxDist;

        // Set the positioning tolerance
        positionTolerance = posTolerance;
    }

    @Override
    public void periodic() {
        if (Parameters.tuningMode) {
            pidController.setP(kP.get());
            pidController.setD(kD.get());
        }
    }

    public void run(double percent) {
        spoolMotor.set(percent);
    }

    public void stop() {
        spoolMotor.stopMotor();
    }
    /**
     * Tells the spool to move to a specific angle
     *
     * @param dist The angle, in degrees, to move the spool to
     */
    public void setDesiredDistance(double dist) {

        // Set the motor's distance if homed
        if (homed) {
            pidController.setReference(dist, controlType);
        } else {
            System.out.println("TUBE NOT HOMED!!!");
        }

        // Print out the angle information if desired
        if (Parameters.debug) {
            System.out.println(
                    String.format("S: %.2f | A: %.2f", dist, spoolMotorEncoder.getPosition()));
        }
    }

    /** Returns the desired distance of the tube */
    public double getDesiredDistance() {

        // Set the tube's distance if homed
        if (homed) {
            return pidController.getReference();
        } else {
            System.out.println("Distancing not available till homed!");
            return 0;
        }
    }

    /**
     * Sets the current distance of the tube. This should be used when homing the tube.
     *
     * @param currentDistance
     */
    public void setCurrentDistance(double currentDistance) {

        // Set the current position
        spoolMotorEncoder.setPosition(currentDistance);

        // Set the soft limits
        // Soft limits are basically the controller not allowing certain values to be set for the
        // PID loop
        spoolMotor.setSoftLimit(SoftLimitDirection.kForward, (float) maxDistance);
        spoolMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) minDistance);

        // Enable the soft limits
        spoolMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        spoolMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        // Set that the spool is homed
        homed = true;
    }

    /**
     * Returns the actual position of the tube
     *
     * @return The position, in m
     */
    public double getCurrentDistance() {
        return spoolMotorEncoder.getPosition();
    }

    /**
     * Checks if the tube is at the desired distance (within tolerance)
     * @return Is at desired distance?
     */
    public boolean isAtDesiredDistance() {
        return (Math.abs(getCurrentDistance() - getDesiredDistance()) < positionTolerance);
    }

    /**
     * Checks if the tube is homed yet
     * @return Is the tube homed yet?
     */
    public boolean isHomed() {
        return homed;
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the tube currently at home?
     */
    public boolean getLSValue() {
        return limitSwitch.get();
    }
}
