// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Parameters;
import frc.robot.utilityClasses.LEDColors;

public class LEDs extends SubsystemBase {

    // Lights! No camera and no action
    private Spark blinkin;
    private double primaryColor = LEDColors.OCEAN;
    private double secondaryColor = LEDColors.OCEAN;
    private double currentColor = 0;
    private boolean alternating = false;
    private Timer timer;

    /** Creates a new LEDs. */
    public LEDs() {

        // Create the Spark object to control the Blinkin
        blinkin = new Spark(Parameters.led.PWM_PORT);

        // Set up the blink timer
        timer = new Timer();

        // Default to the beautiful new blue
        blinkin.set(primaryColor);
    }

    /**
     * Sets the color of the LEDs to the specified value
     *
     * @param colorValue The color to set it to
     */
    public void setPrimaryColor(double colorValue) {
        primaryColor = colorValue;
    }

    /**
     * Sets the color of the LEDs to the specified value
     *
     * @param colorValue The color to set it to
     */
    public void setSecondaryColor(double colorValue) {
        secondaryColor = colorValue;
    }

    /**
     * Sets if the LEDs should alternate between the primary and secondary colors
     *
     * @param alternate Should it alternate?
     */
    public void shouldAlternate(boolean alternate) {
        alternating = alternate;
    }

    /**
     * Sets the new color, but uses caching for better performance
     *
     * @param newColor The new color to set the LEDs to
     */
    private void setColor(double newColor) {

        // Check if the new color is different
        if (newColor != currentColor) {

            // Set the color for the blinkins
            blinkin.set(newColor);

            // Update the current color
            currentColor = newColor;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Check if we should be alternating
        if (alternating) {

            // Check if we need to change the state
            // Note that we need to use half the period to cycle between both states in a full
            // period
            if (timer.hasElapsed(Parameters.led.ALTERNATE_PERIOD / 2)) {

                // Decide the new color for the LEDs
                if (currentColor == primaryColor) {
                    setColor(secondaryColor);
                } else {
                    setColor(primaryColor);
                }

                // Reset the timer
                timer.reset();
            }
        } else {
            // Just set the color to the primary color
            setColor(primaryColor);
        }
    }
}
