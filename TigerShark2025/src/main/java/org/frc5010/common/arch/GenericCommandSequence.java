// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Class for SequentialCommandGroup which provides logging and network table values */
public class GenericCommandSequence extends Command implements WpiHelperInterface {
  /** Prefix for logging, usually the class name */
  protected String logPrefix = getName();
  /** Network table values */
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();
  

  private final List<Command> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  /**
   * Creates a new GenericCommandSequence. The given commands will be run sequentially, with the
   * composition finishing when the last command finishes.
   *
   * @param commands the commands to include in this composition.
   */
  @SuppressWarnings("this-escape")
  public GenericCommandSequence(String log, Command... commands) {
    this.logPrefix = log;
    addCommands(commands);
  }

  public GenericCommandSequence() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.logPrefix = this.getClass().getSimpleName();
    WpiNetworkTableValuesHelper.register(this);
  }

  public GenericCommandSequence(String log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.logPrefix = log;
    WpiNetworkTableValuesHelper.register(this);
  }

  /**
   * Adds the given commands to the group.
   *
   * @param commands Commands to add, in order of execution.
   */
  @SuppressWarnings("PMD.UseArraysAsList")
  public final void addCommands(Command... commands) {
    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      m_commands.add(command);
      addRequirements(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    log(logPrefix + ": Initializing");
    m_currentCommandIndex = 0;

    if (!m_commands.isEmpty()) {
      m_commands.get(0).initialize();
      log(logPrefix + ": Running command " + m_commands.get(m_currentCommandIndex).getName() + " : " + m_currentCommandIndex + " of " + m_commands.size());
    }
  }

  @Override
  public void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    Command currentCommand = m_commands.get(m_currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      m_currentCommandIndex++;
      if (m_currentCommandIndex < m_commands.size()) {
        log(logPrefix + ": Running command " + m_commands.get(m_currentCommandIndex).getName() + " : " + m_currentCommandIndex + " of " + m_commands.size());
        m_commands.get(m_currentCommandIndex).initialize();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted
        && !m_commands.isEmpty()
        && m_currentCommandIndex > -1
        && m_currentCommandIndex < m_commands.size()) {
      m_commands.get(m_currentCommandIndex).end(true);
    }
    m_currentCommandIndex = -1;
    log(logPrefix + ": " + (interrupted ? "Interrupted: " : "Ended: "));
  }

  @Override
  public boolean isFinished() {
    return m_currentCommandIndex == m_commands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, logPrefix);
    builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
  }
}
