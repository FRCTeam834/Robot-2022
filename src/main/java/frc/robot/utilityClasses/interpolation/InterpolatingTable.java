// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilityClasses.interpolation;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BiConsumer;

import frc.robot.Parameters;

import java.util.TreeMap;

/** Add your docs here. */
public class InterpolatingTable {

    // Main table entries
    private TreeMap<Double, ShotParams> table;

    // Printing out a single entry
    private BiConsumer<Double, ShotParams> printEntry = (x, y) -> System.out.println(String.format("Map.entry(%.4f, new ShotParams(%.4f, %.4f),", x, y.angle, y.speed));

    // Main constructor
    public InterpolatingTable() {

        // Initialize the table to a blank table (will eventually be changed to a filled out one)
        table = new TreeMap<Double, ShotParams>();
    }

    /**
     * Adds an entry to the table
     * @param distance The distance to the goal (m)
     * @param parameters The angle and speed of the successful shot
     */
    public void addEntry(Double distance, ShotParams parameters) {
        table.put(distance, parameters);
    }

    /**
     * Adds an entry to the table
     * @param distance The distance to the goal (m)
     * @param angle The angle of the successful shot (deg)
     * @param speed The speed of the successful shot (m/s)
     */
    public void addEntry(Double distance, double angle, double speed) {
        table.put(distance, new ShotParams(angle, speed));
    }

    /**
     * Returns the optimal shot parameters for a given distance
     * @param distance The distance to the goal (m)
     * @return The optimal shot parameters
     */
    public ShotParams getShotParam(double distance) {

        // Get the ceiling and floor entries (the closest on either side)
        Entry<Double, ShotParams> ceiling = table.ceilingEntry(distance);
        Entry<Double, ShotParams> floor = table.floorEntry(distance);

        // Check that the floor and ceiling aren't null
        if (ceiling == null && floor == null) {

            // There are no values in the table, return the default shot parameters
            return new ShotParams(Parameters.hood.DEFAULT_ANGLE, Parameters.shooter.DEFAULT_SPEED);
        }
        else if (ceiling == null) {

            // There's no ceiling (the entry is past the end of the table)
            return floor.getValue();
        }
        else if (floor == null) {

            // There's no floor (the entry is past the start of the table)
            return ceiling.getValue();
        }
        else if (ceiling.getValue().equals(floor.getValue())) {

            // If they're the same, we shouldn't interpolate them
            return ceiling.getValue();
        }
        else {
            // Both the entries are valid, so interpolate the table
            return ceiling.getValue()
                .interpolate(
                        floor.getValue(),
                        (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey()));
        }

    }

    /**
     * Prints out the entire table of shot parameters
     */
    public void printTable() {
        table.forEach(printEntry);
    }
}
