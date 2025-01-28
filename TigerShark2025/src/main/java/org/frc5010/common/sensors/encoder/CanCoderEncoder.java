// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class CanCoderEncoder implements GenericEncoder {

  private CANcoder canCoder;

  /**
   * Constructor for CanCoderEncoder
   *
   * @param CanID - CAN ID
   */
  public CanCoderEncoder(int CanID) {
    this.canCoder = new CANcoder(CanID);
    CANcoderConfiguration config = new CANcoderConfiguration();
    // config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    // config.sensorCoefficient = 360.0 / 4096.0;
    // config.unitString = "degrees";
    // config.sensorTimeBase = SensorTimeBase.PerSecond;
    // canCoder.configAllSettings(config);
  }

  @Override
  public double getPosition() {
    return Units.degreesToRadians(canCoder.getAbsolutePosition().getValue().magnitude());
  }

  @Override
  public double getVelocity() {
    return canCoder.getVelocity().getValue().magnitude();
  }

  @Override
  public void reset() {
    // canCoder.setPositionToAbsolute();
  }

  @Override
  public void setPositionConversion(double conversion) {}

  @Override
  public void setVelocityConversion(double conversion) {}

  @Override
  public void setPosition(double position) {
    canCoder.setPosition(position);
  }

  @Override
  public void setRate(double rate) {}

  @Override
  public void setInverted(boolean inverted) {}

  @Override
  public double getPositionConversion() {
    return 1;
  }

  @Override
  public double getVelocityConversion() {
    return 1;
  }
  
  @Override
  public void simulationUpdate(Optional<Double> position, Double velocity) {}
}
