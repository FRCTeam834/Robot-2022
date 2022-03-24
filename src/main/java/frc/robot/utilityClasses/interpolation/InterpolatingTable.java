package frc.robot.utilityClasses.interpolation;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.BiConsumer;

import frc.robot.Parameters.climber;

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
                                entry(104.0, new ShotParams(26.9, 44.8)),
                                entry(114.2, new ShotParams(27.4, 44.4)),
                                entry(95.2, new ShotParams(25.85, 50.0)),
                                entry(85.0, new ShotParams(23.55, 52.01)),
                                entry(75.89, new ShotParams(23.06, 52.72)),
                                entry(65.0, new ShotParams(22.72, 55.31)),
                                entry(55.16, new ShotParams(22.22, 57.67)),
                                entry(47.28, new ShotParams(21.39, 59.32)),
                                entry(32.45, new ShotParams(20.51, 64.15)),
                                entry(19.71, new ShotParams(20.25, 69.1))));
    }

    public ShotParams getShotParam(double distance) {
        Entry<Double, ShotParams> ceiling = table.ceilingEntry(distance);
        Entry<Double, ShotParams> floor = table.floorEntry(distance);
        if (ceiling == null) {
            return floor.getValue();
        }
        if (floor == null) return ceiling.getValue();
        //if (ceiling.getValue().equals(floor.getValue())) return ceiling.getValue();
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
