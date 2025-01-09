// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

/** Settings that determine the user mode settings of the robot */
public class UserModeJson {
  /** The maximum speed of the robot */
  public double maxSpeed = 0.0;

  /** The maximum angular speed of the robot */
  public double maxAngularSpeed = 0.0;

  /** The maximum acceleration of the robot */
  public double maxAccelleration = 0.0;

  /** The maximum angular acceleration of the robot */
  public double maxAngularAccelleration = 0.0;
}
