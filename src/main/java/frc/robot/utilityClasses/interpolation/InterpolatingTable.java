package frc.robot.utilityClasses.interpolation;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BiConsumer;
import java.util.TreeMap;

/** Add your docs here. */
public class InterpolatingTable {
    private static TreeMap<Double, ShotParams> table;
    private BiConsumer<Double, ShotParams> printEntry = (x, y) -> System.out.println(String.format("Map.entry(%.4f, new ShotParams(%.4f, %.4f),", x, y.angle, y.speed));
    
    public InterpolatingTable() {
        table =
            new TreeMap<>(
                    Map.ofEntries(
                            entry(1.0, new ShotParams(10, 20)),
                            entry(2.0, new ShotParams(30, 40)),
                            entry(3.0, new ShotParams(50, 60)),
                            entry(4.0, new ShotParams(70, 80))));
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

    public static ShotParams getShotParam(double distance) {
        Entry<Double, ShotParams> ceiling = table.ceilingEntry(distance);
        Entry<Double, ShotParams> floor = table.floorEntry(distance);
        if (ceiling == null) {
            return floor.getValue();
        }
        if (floor == null) return ceiling.getValue();
        if (ceiling.getValue().equals(floor.getValue())) return ceiling.getValue();
        return ceiling.getValue()
                .interpolate(
                        floor.getValue(),
                        (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey()));
    }

        /**
     * Prints out the entire table of shot parameters
     */
    public void printTable() {
        table.forEach(printEntry);
    }
}