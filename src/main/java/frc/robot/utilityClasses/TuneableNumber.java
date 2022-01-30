package frc.robot.utilityClasses;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Parameters;

//taken from: https://github.com/Mechanical-Advantage/RobotCode2020/blob/master/src/main/java/frc/robot/util/TunableNumber.java

public class TuneableNumber {
    private String key;
    private double defaultValue;

  /**
   * Create a new TunableNumber
   * 
   */
  public TuneableNumber(String dashboardKey, double defaultValue) {
    this.key = dashboardKey;
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
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   * 
   * @return The current value
   */
  public double get() {
    return Parameters.driver.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
}
}

