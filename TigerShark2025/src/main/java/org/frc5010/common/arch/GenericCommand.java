// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/** Base class for commands that provides default logging and network table support */
public class GenericCommand extends Command implements WpiHelperInterface {
  /** Prefix for logging, usually the class name */
  protected String logPrefix = getName();
  /** Network table values */
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();

  /**
   * Creates a new GenericCommand.
   *
   * @param logPrefix - Prefix for logging
   */
  public GenericCommand(String logPrefix) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.logPrefix = logPrefix;
    WpiNetworkTableValuesHelper.register(this);
  }

  /** Creates a new GenericCommand. */
  public GenericCommand() {
    this.logPrefix = this.getClass().getSimpleName();
    WpiNetworkTableValuesHelper.register(this);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public final void initialize() {
    log(logPrefix + ": Initializing");
    init();
  }

  /** Called when the command is scheduled. Override this to implement the command with logging. */
  public void init() {}

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted - true if the command was interrupted
   */
  @Override
  public final void end(boolean interrupted) {
    stop(interrupted);
    log(logPrefix + ": " + (interrupted ? "Interrupted: " : "Ended: "));
  }

  /**
   * Called once the command ends or is interrupted. Override this to implement the command with
   * logging.
   *
   * @param interrupted - true if the command was interrupted
   */
  public void stop(boolean interrupted) {}

  /** Called when the command is created and registers it with the network tables */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, logPrefix);
  }
}
