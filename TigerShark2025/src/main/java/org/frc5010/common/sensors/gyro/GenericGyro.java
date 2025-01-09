// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.gyro;

/** A generic interface for gyros */
public interface GenericGyro {
  /** Reset the gyro */
  void reset();

  /**
   * Get the angle of the gyro
   *
   * @return the angle
   */
  double getAngle();

  /**
   * Get the x-axis angle of the gyro
   *
   * @return the angle
   */
  double getAngleX();

  /**
   * Get the y-axis angle of the gyro
   *
   * @return the angle
   */
  double getAngleY();

  /**
   * Get the z-axis angle of the gyro
   *
   * @return the angle
   */
  double getAngleZ();

  /**
   * Get the rate of the gyro
   *
   * @return the rate
   */
  double getRate();

  /**
   * Set the angle of the gyro
   *
   * @param angle the angle
   */
  void setAngle(double angle);
}
