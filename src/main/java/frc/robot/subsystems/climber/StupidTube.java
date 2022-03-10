package frc.robot.subsystems.climber;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilityClasses.TuneableNumber;

public class StupidTube extends SubsystemBase {

    // Motor object
    CANSparkMax spoolMotor;

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

    public StupidTube(
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

        // Initialize the spool motor
        spoolMotor = new CANSparkMax(ID, MotorType.kBrushless);
        spoolMotor.restoreFactoryDefaults();
        spoolMotor.setIdleMode(IdleMode.kBrake);
        spoolMotor.setSmartCurrentLimit(20);
        spoolMotor.setInverted(inverted);

        // Set up the limit switch
        limitSwitch = new DigitalInput(LSPort);
    }

    @Override
    public void periodic() {}

    public void set(double percent) {
        spoolMotor.set(percent);
    }

    public void stop() {
        spoolMotor.stopMotor();
    }

    /**
     * Checks if the tube is homed yet
     *
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
        return !limitSwitch.get();
    }

    public void setCurrentLimit(int limit) {
        spoolMotor.setSmartCurrentLimit(limit);
    }
}
