package frc.robot.utilityClasses.interpolation;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static java.util.Map.entry;

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
                                entry(19.3, new ShotParams(19.9, 59.8)),
                                entry(30.6, new ShotParams(20.17, 59.79)),
                                entry(40.05, new ShotParams(20.65, 59.79)),
                                entry(50.6, new ShotParams(21.35, 56.73)),
                                entry(60.1, new ShotParams(22, 55.08)),
                                entry(70.6, new ShotParams(22.66, 54.72)),
                                entry(80.3, new ShotParams(23.57, 51.54)),
                                entry(90.96, new ShotParams(24.43, 49.19)),
                                entry(100.0, new ShotParams(25.59, 46.59)),
                                entry(110.7, new ShotParams(26.7, 43.64))));
    }

    public ShotParams getShotParam(double distance) {
        Entry<Double, ShotParams> ceiling = table.ceilingEntry(distance);
        Entry<Double, ShotParams> floor = table.floorEntry(distance);
        if (ceiling == null) {
            return floor.getValue();
        }
        if (floor == null) return ceiling.getValue();
        // if (ceiling.getValue().equals(floor.getValue())) return ceiling.getValue();
        /*
        System.out.println("Ceil: " + ceiling.toString());
        System.out.println("Floor: " + floor.toString());
        System.out.println("Result: " + distance+  " " + floor.getValue()
        .interpolate(
                ceiling.getValue(),
                (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey())).toString());*/

        return floor.getValue()
                .interpolate(
                        ceiling.getValue(),
                        (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey()));
    }

    /** Prints out the entire table of shot parameters */
    public void printTable() {
        table.forEach(printEntry);
    }
}
