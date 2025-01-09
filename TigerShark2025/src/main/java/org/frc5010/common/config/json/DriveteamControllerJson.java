// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

/**
 * A simple class to store configuration data for a single driveteam controller.
 *
 * <ul>
 *   <li>name: A public field to store the name of a driveteam controller.
 *   <li>port: A public field to store the port number of a driveteam controller.
 *   <li>axis: A public field to store an array of axis file names for a driveteam controller.
 * </ul>
 */
public class DriveteamControllerJson {
  /** The name of the controller */
  public String name;

  /** The port number of the controller */
  public int port;

  /** An array of axis file names for the controller */
  public String[] axis;
}
