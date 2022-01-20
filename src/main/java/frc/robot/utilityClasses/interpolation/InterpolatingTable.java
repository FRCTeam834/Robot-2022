// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilityClasses.interpolation;

import static java.util.Map.entry;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

/** Add your docs here. */
public class InterpolatingTable {
    private InterpolatingTable() {}

    private static final TreeMap<Double, ShotParams> table =
            new TreeMap<>(
                    Map.ofEntries(
                            entry(1.0, new ShotParams(10, 20)),
                            entry(2.0, new ShotParams(10, 20)),
                            entry(3.0, new ShotParams(10, 20)),
                            entry(4.0, new ShotParams(10, 20))));

    public static ShotParams getDistance(double distance) {
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
}