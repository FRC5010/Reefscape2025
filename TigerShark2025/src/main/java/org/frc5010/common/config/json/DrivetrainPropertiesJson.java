// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;
import org.frc5010.common.arch.GenericRobot;

/** Interface for Drivetrain properties classes */
public interface DrivetrainPropertiesJson {
  /** Drivetrain subsystem key name */
  public static String DRIVE_TRAIN = "DriveTrain";
  /**
   * Reads the drivetrain configuration from the given directory
   *
   * @param robot the robot being configured
   * @param directory the directory to read from
   * @throws IOException if something fails on the filesystem
   */
  public void readDrivetrainConfiguration(GenericRobot robot, File directory) throws IOException;

  /**
   * Creates the drivetrain
   *
   * @param robot the robot being configured
   */
  public void createDriveTrain(GenericRobot robot);
}
