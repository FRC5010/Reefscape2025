// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import org.frc5010.common.telemetry.WpiDataLogging;

/** An interface for logging that can be mixed in classes */
public interface WpiHelperInterface {
  /**
   * Logs a message to WPI logger
   *
   * @param logMessage - message to log
   */
  default void log_wpi(String logMessage) {
    WpiDataLogging.log(this.getClass().getSimpleName() + ": " + logMessage);
  }

  /**
   * Logs a message to the RIO Log
   *
   * @param logMessage - message to log
   */
  default void log_rio(String logMessage) {
    System.out.println(this.getClass().getSimpleName() + ": " + logMessage);
  }

  /**
   * Logs to the WPI logger and the Rio log
   *
   * @param logMessage - message to log
   */
  default void log(String logMessage) {
    log_wpi(logMessage);
    log_rio(logMessage);
  }
}
