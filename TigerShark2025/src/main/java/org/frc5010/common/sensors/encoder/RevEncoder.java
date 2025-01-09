// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.EncoderConfig;

public class RevEncoder implements GenericEncoder {
  RelativeEncoder encoder;
  EncoderConfig config;

  public RevEncoder(RelativeEncoder encoder) {
    this.encoder = encoder;
    config = new EncoderConfig();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void reset() {
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public void setRate(double rate) {
    throw new UnsupportedOperationException("Not supported for RevEncoder");
  }

  @Override
  public void setPositionConversion(double conversion) {
    config.positionConversionFactor(conversion);
  }

  @Override
  public void setVelocityConversion(double conversion) {
    config.velocityConversionFactor(conversion);
  }

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
  }
}
