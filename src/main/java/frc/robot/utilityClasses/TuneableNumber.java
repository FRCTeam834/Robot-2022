package frc.robot.utilityClasses;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import frc.robot.Parameters;

// taken from:
// https://github.com/Mechanical-Advantage/RobotCode2020/blob/master/src/main/java/frc/robot/util/TunableNumber.java

public class TuneableNumber {
    private NetworkTableEntry entry;
    private double defaultValue;

    /** Create a new TunableNumber */
    public TuneableNumber(NetworkTable table, String name, double defaultValue) {
        this.entry = table.getEntry(name);
        setDefault(defaultValue);
    }

    /**
     * Get the default value for the number that has been set
     *
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     *
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (Parameters.driver.tuningMode) {
            // This makes sure the data is on NetworkTables but will not change it
            entry.setDouble(defaultValue);
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     *
     * @return The current value
     */
    public double get() {
        return Parameters.driver.tuningMode
                ? entry.getDouble(defaultValue)
                : defaultValue;
    }
}
