// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.sensors.Controller;

/**
 * GenericMechanism should be used as the parent class of any mechanism It
 * enforces the use of the
 * functions:
 */
public abstract class GenericMechanism implements WpiHelperInterface, Sendable {
  /** The visual representation of the mechanism */
  protected Mechanism2d mechVisual;
  /** The tab for the mechanism */
  protected ShuffleboardTab shuffleTab;
  /** The network table values */
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();
  /** The log prefix */
  protected String logPrefix = getClass().getSimpleName();

  /**
   * Constructor for GenericMechanism
   *
   * @param tabName - the name of the display tab
   */
  public GenericMechanism() {
    this.mechVisual = new Mechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV);
    shuffleTab = Shuffleboard.getTab(logPrefix);
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Constructor for GenericMechanism
   *
   * @param tabName - the name of the display tab
   */
  public GenericMechanism(String tabName) {
    this.mechVisual = new Mechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV);
    shuffleTab = Shuffleboard.getTab(tabName);
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Constructor for GenericMechanism with specified visual
   *
   * @param robotMechVisual - the visual representation of the mechanism
   * @param shuffleTab      - the tab for the mechanism
   */
  public GenericMechanism(Mechanism2d robotMechVisual, ShuffleboardTab shuffleTab) {
    this.mechVisual = robotMechVisual;
    this.shuffleTab = shuffleTab;
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Retrieves the ShuffleboardTab associated with this GenericMechanism.
   *
   * @return the ShuffleboardTab for this GenericMechanism
   */
  public ShuffleboardTab getDisplayTab() {
    return shuffleTab;
  }

  /**
   * Adds a variable to the Display in a list
   *
   * @param key - String name of the variable being stored.
   */
  public void addToTab(String key) {
    values.addToTab(shuffleTab, key);
  }

  /**
   * Adds a variable to the Display in a list.
   *
   * @param list the name of the list to add the variable to
   * @param key  the name of the variable being stored
   */
  public void addToTabList(String list, String key) {
    values.addToTabList(shuffleTab, list, key);
  }

  /**
   * configureButtonBindings should map button/axis controls to commands
   *
   * @param driver   - driver joystick
   * @param operator - operator joystick
   */
  public abstract void configureButtonBindings(Controller driver, Controller operator);

  /**
   * setupDefaultCommands should setup the default commands needed by subsystems
   * It could check for
   * Test mode and enable different commands
   *
   * @param driver   - driver joystick
   * @param operator - operator joystick
   */
  public abstract void setupDefaultCommands(Controller driver, Controller operator);

  /**
   * Sets up the default commands for testing purposes.
   *
   * @param driver   the driver controller
   * @param operator the operator controller
   */
  public void setupTestDefaultCommmands(Controller driver, Controller operator) {
  }

  /**
   * initRealOrSim should check the real or simulation state of the robot and
   * initialize its code
   * accordingly
   */
  protected abstract void initRealOrSim();

  /** setupPreferences should be implemented in place of using Constants files */
  protected void setupPreferences() {
  }

  /** Used to initialize auto commands for the robot */
  public abstract void initAutoCommands();

  /**
   * Used to wrap the selected auto command in additional behavior
   *
   * @param autoCommand the auto command
   * @return the wrapped auto command
   */
  public abstract Command generateAutoCommand(Command autoCommand);

  /** Executed periodically when robot is disabled */
  public void disabledBehavior() {
  }

  /**
   * Initializes the sendable builder with the necessary components for this
   * class.
   *
   * @param builder the sendable builder to initialize
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, this.getClass().getSimpleName());
  }
}
