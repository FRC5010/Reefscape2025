// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import java.util.function.BooleanSupplier;

import com.thethriftybot.ThriftyNova;

/** Add your docs here. */
public class ThriftyLimit {
    ThriftyNova motor;

    public ThriftyLimit(ThriftyNova motor) {
        this.motor = motor;
    }

    public boolean getForward() {
        return getForwardBooleanSupplier().getAsBoolean();
    }

    public BooleanSupplier getForwardBooleanSupplier() {
        return motor.getForwardLimit();
    }

    public boolean getReverse() {
        return getReverseBooleanSupplier().getAsBoolean();
    }

    public BooleanSupplier getReverseBooleanSupplier() {
        return motor.getReverseLimit();
    }
}
