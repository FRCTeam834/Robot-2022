package frc.robot.utilityClasses.interpolation;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static java.util.Map.entry;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.BiConsumer;

/** Add your docs here. */
public class InterpolatingTable {
    
    private static TreeMap<Double, ShotParams> table;
    private BiConsumer<Double, ShotParams> printEntry =
            (x, y) ->
                    System.out.println(
                            String.format(
                                    "Map.entry(%.4f, new ShotParams(%.4f, %.4f),",
                                    x, y.angle, y.speed));

    public InterpolatingTable() {
        table =
                new TreeMap<>(
                        Map.ofEntries(
                                entry(0.0, new ShotParams(0, 0)),
                                entry(1.32, new ShotParams(18.39, 70.16)),
                                entry(1.75, new ShotParams(21.39, 71.1)),
                                entry(2.413, new ShotParams(21.95, 63.211)),
                                entry(3.35, new ShotParams(22.19, 59.44)),
                                entry(3.749, new ShotParams(22.49,57.44)),
                                entry(4.31, new ShotParams(22.76,55.19)),
                                entry(4.89, new ShotParams(23.59,55.079))));

    }

    public ShotParams getShotParam(double distance) {
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

    /** Prints out the entire table of shot parameters */
    public void printTable() {
        table.forEach(printEntry);
    }
}
