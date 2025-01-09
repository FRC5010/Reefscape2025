// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

/** A Generic encoder interface */
public interface GenericEncoder {
  /**
   * Get the current position of the encoder
   *
   * @return double
   */
  double getPosition();

  /**
   * Get the current velocity of the encoder
   *
   * @return double
   */
  double getVelocity();

  /** Reset the encoder */
  void reset();

  /**
   * Set the position conversion factor of the encoder
   *
   * @param conversion - multiplier for position conversion
   */
  void setPositionConversion(double conversion);

  /**
   * Set the velocity conversion factor of the encoder
   *
   * @param conversion - multiplier for velocity conversion
   */
  void setVelocityConversion(double conversion);

  /**
   * Set the position of the encoder
   *
   * @param position - position in native units
   */
  void setPosition(double position);

  /**
   * Set the rate of the encoder
   *
   * @param rate - rate in native units per second
   */
  void setRate(double rate);

  /**
   * Set the inversion state of the encoder
   *
   * @param inverted - true if encoder should be inverted
   */
  void setInverted(boolean inverted);
}
