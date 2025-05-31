// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Beambreak {
    BooleanSupplier valueSupplier;

    public Beambreak(int channel) {
        valueSupplier = () -> false;
    }

    public Beambreak(BooleanSupplier supplier) {
        valueSupplier = supplier;
    }

    public boolean isBroken() {
        return valueSupplier.getAsBoolean();
    }

    public BooleanSupplier isBrokenSupplier() {
        return () -> isBroken();
    }
}
