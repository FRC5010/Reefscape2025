// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class FieldDimensions {
    public static final FieldDimensions REEFSCAPE = new FieldDimensions(Inches.of(690.876), Inches.of(317), true, true);

    public final Distance fieldWidth;
    public final Distance fieldLength;
    public final Boolean xFlip;
    public final Boolean yFlip;

    public FieldDimensions(Distance length, Distance width, Boolean xFlip, Boolean yFlip) {
        this.fieldWidth = width;
        this.fieldLength = length;
        this.xFlip = xFlip;
        this.yFlip = yFlip;
    }
    
}
