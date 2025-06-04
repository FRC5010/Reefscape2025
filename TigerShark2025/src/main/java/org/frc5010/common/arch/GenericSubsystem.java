// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.motors.function.GenericFunctionalMotor;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for subsystems that provides default logging and network table
 * support
 */
public class GenericSubsystem extends SubsystemBase
    implements WpiHelperInterface, GenericDeviceHandler {
  /** The network table values */
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();
  protected final DisplayValuesHelper displayValues;
  /** The log prefix */
  protected String logPrefix = getClass().getSimpleName();
  /** The mechanism simulation */
  protected LoggedMechanism2d mechanismSimulation;
  /** The map of devices created by the configuration system */
  protected Map<String, Object> devices = new HashMap<>();
  protected Alert loggingAlert = new Alert(logPrefix + " Logging Mode is not COMPETITION!", AlertType.kWarning);

  /** Creates a new LoggedSubsystem. */
  public GenericSubsystem(LoggedMechanism2d mechanismSimulation) {
    this.mechanismSimulation = mechanismSimulation;
    displayValues = new DisplayValuesHelper(logPrefix, logPrefix);
    WpiNetworkTableValuesHelper.register(this);
  }

  public GenericSubsystem() {
    displayValues = new DisplayValuesHelper(logPrefix, logPrefix);
    WpiNetworkTableValuesHelper.register(this);
  }

  public GenericSubsystem(String configFile) {
    displayValues = new DisplayValuesHelper(logPrefix, logPrefix);
    try {
      GenericRobot.subsystemParser.parseSubsystem(this, configFile);
    } catch (Exception e) {
      e.printStackTrace();
      System.exit(1);
    }
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Get the mechanism simulation visual
   *
   * @return the mechanism visual
   */
  public LoggedMechanism2d getMechVisual() {
    return mechanismSimulation;
  }

  /**
   * Set the mechanism simulation visual
   *
   * @param mechSim the mechanism visual
   */
  public void setMechSimulation(LoggedMechanism2d mechSim) {
    mechanismSimulation = mechSim;
  }

  /**
   * Add a device to the configuration
   *
   * @param name   the name of the device
   * @param device the device
   */
  @Override
  public void addDevice(String name, Object device) {
    devices.put(name, device);
  }

  public void setLoggingLevel(LogLevel logLevel) {
    displayValues.setLoggingLevel(logLevel);
    if (logLevel == LogLevel.COMPETITION) {
      loggingAlert.set(false);;
    } else {
      loggingAlert.set(true);
    }
  }
  /**
   * Get a device from the configuration
   *
   * @param name the name of the device
   * @return the device
   */
  @Override
  public Object getDevice(String name) {
    return devices.get(name);
  }

  /**
   * Called when the command is created and registers it with the network tables
   *
   * @param builder - The sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, this.getClass().getSimpleName());
  }

  /**
   * Called every time the scheduler runs while the robot is enabled. Used to
   * update display values and draw motor graphics.
   */
  @Override
  public void periodic() {
    devices.values().stream()
        .forEach(
            it -> {
              if (it instanceof GenericFunctionalMotor) {
                ((GenericFunctionalMotor) it).periodicUpdate();
              }
            });
  }

  /**
   * Called every time the scheduler runs while the robot is in simulation mode.
   * Used to update
   * simulation models.
   */
  @Override
  public void simulationPeriodic() {
    devices.values().stream()
        .forEach(
            it -> {
              if (it instanceof GenericFunctionalMotor) {
                ((GenericFunctionalMotor) it).simulationUpdate();
              }
            });
  }

  /**
   * Get the display values helper associated with the subsystem.
   *
   * @return the DisplayValuesHelper instance for managing display values
   */
  @Override
  public DisplayValuesHelper getDisplayValuesHelper() {
    return displayValues;
  }

  /**
   * Sets the display state of the subsystem.
   *
   * @param display a boolean indicating whether to enable or disable the display
   */
  public void setDisplay(boolean display) {
    if (display) 
      displayValues.makeDisplayed();
  }
}
