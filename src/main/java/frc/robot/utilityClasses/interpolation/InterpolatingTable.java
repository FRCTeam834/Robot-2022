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
                                entry(0.0, new ShotParams(19.49, 76.76)),
                                entry(101.1, new ShotParams(28.63, 45.41)),
                                entry(114.2, new ShotParams(27.24, 46.4)),
                                entry(95.2, new ShotParams(26.95, 48.55)),
                                entry(90.63, new ShotParams(24.8667, 47.5)),
                                entry(81.89, new ShotParams(25, 49.65)),
                                entry(70.81, new ShotParams(24.26, 49.89)),
                                entry(59.58, new ShotParams(23.01, 53.19)),
                                entry(44.66, new ShotParams(22, 55.55)),
                                entry(32.45, new ShotParams(21.25, 62.15)),
                                entry(27.7, new ShotParams(20.93, 60.147)),
                                entry(18.48, new ShotParams(20.45, 63.144))));
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
