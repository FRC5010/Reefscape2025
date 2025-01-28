// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import java.util.Optional;

import com.thethriftybot.Conversion;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.EncoderType;

/** Add your docs here. */
public class ThriftyNovaEncoder implements GenericEncoder {
    protected ThriftyNova encoder;
    protected Conversion positionConversion = new Conversion(
            PositionUnit.ROTATIONS, EncoderType.INTERNAL);
    protected Conversion velocityConversion = new Conversion(
            VelocityUnit.ROTATIONS_PER_MIN, EncoderType.INTERNAL);
    protected double rotationConversion = 1.0;
    protected double speedConversion = 1.0 / 60.0;

    public ThriftyNovaEncoder(ThriftyNova encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getPosition() {
        return positionConversion.fromMotor(encoder.getPosition()) * rotationConversion;
    }

    @Override
    public double getVelocity() {
        return velocityConversion.fromMotor(encoder.getVelocity()) * (rotationConversion / 60.0);
    }

    @Override
    public void reset() {
        encoder.setEncoderPosition(0);
    }

    @Override
    public void setPositionConversion(double conversion) {
        rotationConversion = conversion;
    }

    @Override
    public void setVelocityConversion(double conversion) {
        speedConversion = conversion;
    }

    @Override
    public void setPosition(double position) {
        encoder.setEncoderPosition(positionConversion.toMotor(position / rotationConversion));
    }

    @Override
    public void setRate(double rate) {
    }

    @Override
    public void setInverted(boolean inverted) {
        encoder.setInversion(inverted);
    }

    @Override
    public double getPositionConversion() {
        return rotationConversion;
    }

    @Override
    public double getVelocityConversion() {
        return speedConversion;
    }

    @Override
    public void simulationUpdate(Optional<Double> position, Double velocity) {
    }
}
