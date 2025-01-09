// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** A class for specifying drive ports */
public class DrivePorts {
  /** The drive port */
  protected int drivePort;

  /**
   * Constructor for DrivePorts
   *
   * @param drivePort the drive port
   */
  public DrivePorts(int drivePort) {
    this.drivePort = drivePort;
  }

  /**
   * Returns the drive port
   *
   * @return the drive port
   */
  public int getDrivePort() {
    return drivePort;
  }

  /**
   * Sets the drive port
   *
   * @param drivePort the drive port
   */
  public void setDrivePort(int drivePort) {
    this.drivePort = drivePort;
  }
}
