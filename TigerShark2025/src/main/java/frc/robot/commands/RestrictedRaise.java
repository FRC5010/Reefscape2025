// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RestrictedRaise extends Command {
  private ElevatorSystem elevator;
  private Supplier<Distance> maxHeight;
  private Supplier<Distance> goal;
  private Distance prevDriveTo = Meters.zero();
  /** Creates a new RestrictedRaise. */
  public RestrictedRaise(ElevatorSystem elevator, Supplier<Distance> maxHeight, Supplier<Distance> goal) {
    this.elevator = elevator;
    this.maxHeight = maxHeight;
    this.goal = goal;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Distance getCappedGoal() {
    return goal.get().lt(maxHeight.get()) ? goal.get() : maxHeight.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetController(goal.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Distance driveTo = getCappedGoal();

    if (!driveTo.isNear(prevDriveTo, Meters.of(0.005))) {
      elevator.setControllerGoal(driveTo);
      prevDriveTo = driveTo;
    }

    elevator.runControllerToSetpoint();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
