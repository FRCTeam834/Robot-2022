/** @author Christian Piper (@CAP1Sup) */
package frc.robot.utilityClasses;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class CachedPIDController {

    // Variables for caching
    double previousValue = 0;
    ControlType previousControlType = ControlType.kDutyCycle;
    REVLibError previousREVLibError = REVLibError.kOk;
    SparkMaxPIDController pidController;

    /**
     * Creates a new Cached Spark Max object. This is useful for preventing the sending of repeated
     * values to the Spark Maxes. It will help to reduce the load on the CAN bus (only a set amount
     * of information can be sent across the CAN bus in a given time).
     *
     * @param deviceID The CAN ID of the motor controller
     * @param type The type of motor being used
     */
    @SuppressWarnings("all")
    public CachedPIDController(CANSparkMax device) {
        // ! IGNORE THIS WARNING, there's no other way to create the new
        // controller. Even REV themselves use it
        pidController = device.getPIDController();
    }

    /**
     * Sets the reference with caching (repeated values are not set)
     *
     * @param value The value to set depending on the control mode. For basic duty cycle control
     *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
     *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
     *     Current (Amps). Native units can be changed using the setPositionConversionFactor() or
     *     setVelocityConversionFactor() methods of the RelativeEncoder class
     * @param ctrl is the control type
     * @return Set to REV_OK if successful
     */
    public REVLibError setReference(double value, ControlType ctrl) {

        // Check if the values have been changed
        // We use separated if loops to reduce the typical amount of checks needed
        // Values change the most, so we check those first, then control types
        if (value != previousValue) {
            if (!ctrl.equals(previousControlType)) {

                // We need to send the values
                // First save the current values for the next cycle
                previousValue = value;
                previousControlType = ctrl;

                // Send the set command, saving the output
                previousREVLibError = pidController.setReference(value, ctrl);
            }
        }

        // Return the current CAN error
        // We can just return the previous error each time as the variable is updated
        // when unique values are sent
        return previousREVLibError;
    }

    /**
     * Set the Proportional Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
     * Parameter API and should be used infrequently. The parameter does not presist unless
     * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
     * GUI to tune and save parameters.
     *
     * @param gain The proportional gain value, must be positive
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setP(double gain) {
        return pidController.setP(gain);
    }

    /**
     * Set the Integral Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
     * Parameter API and should be used infrequently. The parameter does not presist unless
     * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
     * GUI to tune and save parameters.
     *
     * @param gain The integral gain value, must be positive
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setI(double gain) {
        return pidController.setI(gain);
    }

    /**
     * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
     * Parameter API and should be used infrequently. The parameter does not presist unless
     * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
     * GUI to tune and save parameters.
     *
     * @param gain The derivative gain value, must be positive
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setD(double gain) {
        return pidController.setD(gain);
    }

    /**
     * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX. This uses the Set
     * Parameter API and should be used infrequently. The parameter does not presist unless
     * burnFlash() is called.
     *
     * @param gain The derivative filter value, must be a positive number between 0 and 1
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setDFilter(double gain) {
        return pidController.setDFilter(gain);
    }

    /**
     * Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX. This uses the Set
     * Parameter API and should be used infrequently. The parameter does not presist unless
     * burnFlash() is called. The recommended method to configure this parameter is use to SPARK MAX
     * GUI to tune and save parameters.
     *
     * @param gain The feed-forward gain value
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setFF(double gain) {
        return pidController.setFF(gain);
    }

    /**
     * Set the IZone range of the PIDF controller on the SPARK MAX. This value specifies the range
     * the |error| must be within for the integral constant to take effect.
     *
     * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
     * presist unless burnFlash() is called. The recommended method to configure this parameter is
     * to use the SPARK MAX GUI to tune and save parameters.
     *
     * @param IZone The IZone value, must be positive. Set to 0 to disable
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setIZone(double IZone) {
        return pidController.setIZone(IZone);
    }

    /**
     * Set the min amd max output for the closed loop mode.
     *
     * <p>This uses the Set Parameter API and should be used infrequently. The parameter does not
     * presist unless burnFlash() is called. The recommended method to configure this parameter is
     * to use the SPARK MAX GUI to tune and save parameters.
     *
     * @param min Reverse power minimum to allow the controller to output
     * @param max Forward power maximum to allow the controller to output
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setOutputRange(double min, double max) {
        return pidController.setOutputRange(min, max);
    }

    /**
     * Get the Proportional Gain constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double P Gain value
     */
    public double getP() {
        return pidController.getP();
    }

    /**
     * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double I Gain value
     */
    public double getI() {
        return pidController.getI();
    }

    /**
     * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double D Gain value
     */
    public double getD() {
        return pidController.getD();
    }

    /**
     * Get the Derivative Filter constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double D Filter value
     */
    public double getDFilter() {
        return pidController.getDFilter(0);
    }

    /**
     * Get the Feed-forward Gain constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double F Gain value
     */
    public double getFF() {
        return pidController.getFF();
    }

    /**
     * Get the IZone constant of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double IZone value
     */
    public double getIZone() {
        return pidController.getIZone();
    }

    /**
     * Get the min output of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double min value
     */
    public double getOutputMin() {
        return pidController.getOutputMin();
    }

    /**
     * Get the max output of the PIDF controller on the SPARK MAX.
     *
     * <p>This uses the Get Parameter API and should be used infrequently. This function uses a
     * non-blocking call and will return a cached value if the parameter is not returned by the
     * timeout. The timeout can be changed by calling SetCANTimeout(int milliseconds)
     *
     * @return double max value
     */
    public double getOutputMax() {
        return pidController.getOutputMax();
    }

    /**
     * Configure the maximum velocity of the SmartMotion mode. This is the velocity that is reached
     * in the middle of the profile and is what the motor should spend most of its time at
     *
     * @param maxVel The maxmimum cruise velocity for the motion profile in RPM
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartMotionMaxVelocity(double maxVel) {
        return pidController.setSmartMotionMaxVelocity(maxVel, 0);
    }

    /**
     * Configure the maximum acceleration of the SmartMotion mode. This is the accleration that the
     * motor velocity will increase at until the max velocity is reached
     *
     * @param maxAccel The maxmimum acceleration for the motion profile in RPM per second
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartMotionMaxAccel(double maxAccel) {
        return pidController.setSmartMotionMaxAccel(maxAccel, 0);
    }

    /**
     * Configure the mimimum velocity of the SmartMotion mode. Any requested velocities below this
     * value will be set to 0.
     *
     * @param minVel The minimum velocity for the motion profile in RPM
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartMotionMinOutputVelocity(double minVel) {
        return pidController.setSmartMotionMinOutputVelocity(minVel, 0);
    }

    /**
     * Configure the allowed closed loop error of SmartMotion mode. This value is how much deviation
     * from your setpoint is tolerated and is useful in preventing oscillation around your setpoint.
     *
     * @param allowedErr The allowed deviation for your setpoint vs actual position in rotations
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr) {
        return pidController.setSmartMotionAllowedClosedLoopError(allowedErr, 0);
    }

    /**
     * NOTE: As of the 2022 FRC season, the firmware only supports the trapezoidal motion profiling
     * acceleration strategy.
     *
     * <p>Configure the acceleration strategy used to control acceleration on the motor.
     *
     * @param accelStrategy The acceleration strategy to use for the automatically generated motion
     *     profile
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSmartMotionAccelStrategy(AccelStrategy accelStrategy) {
        return pidController.setSmartMotionAccelStrategy(accelStrategy, 0);
    }

    /**
     * Get the maximum velocity of the SmartMotion mode. This is the velocity that is reached in the
     * middle of the profile and is what the motor should spend most of its time at
     *
     * @return The maxmimum cruise velocity for the motion profile in RPM
     */
    public double getSmartMotionMaxVelocity() {
        return pidController.getSmartMotionMaxVelocity(0);
    }

    /**
     * Get the maximum acceleration of the SmartMotion mode. This is the accleration that the motor
     * velocity will increase at until the max velocity is reached
     *
     * @return The maxmimum acceleration for the motion profile in RPM per second
     */
    public double getSmartMotionMaxAccel() {
        return pidController.getSmartMotionMaxAccel(0);
    }

    /**
     * Get the mimimum velocity of the SmartMotion mode. Any requested velocities below this value
     * will be set to 0.
     *
     * @return The minimum velocity for the motion profile in RPM
     */
    public double getSmartMotionMinOutputVelocity() {
        return pidController.getSmartMotionMinOutputVelocity(0);
    }

    /**
     * Get the allowed closed loop error of SmartMotion mode. This value is how much deviation from
     * your setpoint is tolerated and is useful in preventing oscillation around your setpoint.
     *
     * @return The allowed deviation for your setpoint vs actual position in rotations
     */
    public double getSmartMotionAllowedClosedLoopError() {
        return pidController.getSmartMotionAllowedClosedLoopError(0);
    }

    /**
     * Get the acceleration strategy used to control acceleration on the motor. As of the 2022 FRC
     * season, the strategy is always trapezoidal motion profiling, regardless of what the device
     * may report.
     *
     * @return The acceleration strategy to use for the automatically generated motion profile.
     */
    public AccelStrategy getSmartMotionAccelStrategy() {
        return pidController.getSmartMotionAccelStrategy(0);
    }
}
