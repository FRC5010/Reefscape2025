// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class SparkLimit {
    private SparkMax motor;

    public SparkLimit(SparkMax motor) {
        this.motor = motor;
    }

    public boolean getForwardLimit() {
        return motor.getForwardLimitSwitch().isPressed();
    }

    public boolean getReverseLimit() {
        return motor.getReverseLimitSwitch().isPressed();
    }
}
