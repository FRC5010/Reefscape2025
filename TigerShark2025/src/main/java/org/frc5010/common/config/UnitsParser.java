// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A class that converts a magnitude and a unit into a {@link Measurement}
 * object.
 */
public class UnitsParser {

    /**
     * Converts a magnitude and a unit into a {@link Distance} object.
     * 
     * @param magnitude The magnitude of the distance.
     * @param unit      The unit of the distance. Can be "meters", "feet", "inches",
     *                  "mm", "cm", or "yd".
     *                  (case insensitive)
     * @return The {@link Distance} object.
     */
    public static Distance parseDistance(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case "m":
            case "meter":
            case "meters":
                return Meters.of(magnitude);
            case "in":
            case "inch":
            case "inches":
                return Inches.of(magnitude);
            case "ft":
            case "foot":
            case "feet":
                return Feet.of(magnitude);
            case "mm":
            case "millimeter":
            case "millimeters":
                return Millimeters.of(magnitude);
            case "cm":
            case "centimeter":
            case "centimeters":
                return Centimeters.of(magnitude);
            case "yd":
            case "yard":
            case "yards":
                return Feet.of(magnitude * 3);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to meters");
                break;
        }
        return Meters.of(magnitude);
    }

    public static LinearVelocity parseVelocity(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case "m/s":
            case "m/sec":
            case "meter/sec":
            case "meters/sec":
            case "meter/second":
            case "meters/second":
                return MetersPerSecond.of(magnitude);
            case "in/s":
            case "in/sec":
            case "inch/sec":
            case "inches/sec":
            case "inch/second":
            case "inches/second":
                return InchesPerSecond.of(magnitude);
            case "ft/sec":
            case "ft/s":
            case "foot/sec":
            case "feet/sec":
            case "foot/second":
            case "feet/second":
                return FeetPerSecond.of(magnitude);
            case "mm/s":
            case "mm/sec":
            case "millimeter/sec":
            case "millimeters/sec":
            case "millimeter/second":
            case "millimeters/second":
                return MetersPerSecond.of(magnitude * 1000);
            case "cm/s":
            case "cm/sec":
            case "centimeter/sec":
            case "centimeters/sec":
            case "centimeter/second":
            case "centimeters/second":
                return MetersPerSecond.of(magnitude * 100);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to meters/second.");
                break;
        }
        return MetersPerSecond.of(magnitude);
    }
}
