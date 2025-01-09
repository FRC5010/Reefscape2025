// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** A class that will run for a specified amount of time */
public class ElapseTime extends Command {
  /** the Timer. */
  Timer timer = new Timer();
  /** the time to elapse */
  private double sec;

  /**
   * Creates a new ElapseTime.
   *
   * @param sec the time to elapse
   */
  public ElapseTime(double sec) {
    timer.start();
    this.sec = sec;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.advanceIfElapsed(sec);
  }
}
