// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

/**
 * A simple Java class used for storing configuration data for a single axis of a controller.
 *
 * <ul>
 *   <li>channel: The channel number of the axis on the controller.
 *   <li>deadband: The mininal value of the axis to register motion.
 *   <li>invert: Whether the axis should be inverted.
 *   <li>scale: The scaling factor for the axis.
 *   <li>curvePower: The mathematical power to apply to the axis to create a curved response.
 *   <li>limit: The absolute maximum limit value for the axis.
 *   <li>rate: Optional. The slew rate for the axis.
 * </ul>
 */
public class DriveteamControllerAxisJson {
  /** The channel number of the axis on the controller */
  public int channel;

  /** The mininal value of the axis to register motion */
  public double deadband = 0.0;

  /** Whether the axis should be inverted */
  public boolean invert = false;

  /** The scaling factor for the axis */
  public double scale = 1.0;

  /** The mathematical power to apply to the axis to create a curved response */
  public double curvePower = 1.0;

  /** The absolute maximum limit value for the axis */
  public double limit = 1.0;

  /** Optional. The slew rate for the axis */
  public double rate = 0.0;
}
