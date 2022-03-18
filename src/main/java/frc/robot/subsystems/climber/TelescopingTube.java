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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
            double posTolerance,
            boolean inverted) {

        // Get the table for tubes
        NetworkTable tubeTable = NetworkTableInstance.getDefault().getTable("TeleTubies");

        // Set up the tunable numbers
        this.kP = new TuneableNumber(tubeTable.getSubTable(name), name, kP);
        this.kD = new TuneableNumber(tubeTable.getSubTable(name), name, kD);

        // Initialize the spool motor
        spoolMotor = new CANSparkMax(ID, MotorType.kBrushless);
        spoolMotor.restoreFactoryDefaults();
        spoolMotor.setIdleMode(IdleMode.kBrake);
        spoolMotor.setSmartCurrentLimit(10);
        spoolMotor.setInverted(inverted);

        // Set up the encoder of the spool motor
        spoolMotorEncoder = spoolMotor.getEncoder();
        spoolMotorEncoder.setPositionConversionFactor(circumference / gearboxRatio);
        spoolMotorEncoder.setVelocityConversionFactor(circumference / (gearboxRatio * 60));

        // Set up the PID controller
        pidController = new CachedPIDController(spoolMotor);
        pidController.setOutputRange(-maxDuty, maxDuty);
        pidController.setP(30);
        pidController.setD(0);

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
    public void periodic() {}

    public void set(double percent) {
        spoolMotor.set(percent);
    }

    public void setWithLimitSwitch(double percent) {
        if (!getLSValue()) {
            spoolMotor.set(percent);
        } else if (percent > 0) {
            spoolMotor.set(percent);
        } else {
            spoolMotor.set(0);
        }
    }

    public double getTubePosition() {
        return spoolMotorEncoder.getPosition();
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

        // if not home, home
        if (!homed) {
            CommandScheduler.getInstance().schedule(new HomeClimberTubes());
        } else if (homed) {
            pidController.setReference(dist, controlType);
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
     *
     * @return Is at desired distance?
     */
    public boolean isAtDesiredDistance() {
        return (Math.abs(getCurrentDistance() - getDesiredDistance()) < positionTolerance);
    }

    /**
     * Checks if the tube is homed yet
     *
     * @return Is the tube homed yet?
     */
    public boolean isHomed() {
        return homed;
    }

    public void setDuty(double duty) {
        pidController.setOutputRange(-duty, duty);
    }

    /**
     * Gets if the limit switch is triggered
     *
     * @return Is the tube currently at home?
     */
    public boolean getLSValue() {
        return !limitSwitch.get();
    }

    public void setCurrentLimit(int limit) {
        spoolMotor.setSmartCurrentLimit(limit);
    }
}
