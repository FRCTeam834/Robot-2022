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
                                entry(17.85, new ShotParams(19.98,65.804)),
                                entry(29.7, new ShotParams(20.17,63.68)),
                                entry(41.17, new ShotParams(20.37,60.85)),
                                entry(46.48, new ShotParams(20.93,57.43)),
                                entry(61.32, new ShotParams(22.22,55.78)),
                                entry(69.37, new ShotParams(22.4,54)),
                                entry(77.908, new ShotParams(23.026,54)),
                                entry(85.86, new ShotParams(24,52.6)),
                                entry(92.064, new ShotParams(24.69,50.36)),
                                entry(97.19, new ShotParams(24.79,50.19)),
                                entry(104.91, new ShotParams(25.68,44.94)),
                                //entry(110.84, new ShotParams(26.18,46.17)),
                                entry(113.17, new ShotParams(29,40))));
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
