// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Beambreak extends DigitalInput {
    public Beambreak(int channel) {
        super(channel);
    }

    public boolean isBroken() {
        return !get();
    }

    public BooleanSupplier isBrokenSupplier() {
        return () -> isBroken();
    }
}
