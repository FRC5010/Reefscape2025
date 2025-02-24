// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.subsystems.ElevatorSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdvancedCoralSequence extends GenericCommand {
  ProfiledPIDController elevatorController;
  Command driveToPose;
  /** Creates a new AdvancedCoralSequence. */
  public AdvancedCoralSequence(Pose2d scorePose, ScoringLevel scoringLevel, YAGSLSwerveDrivetrain drivetrain, ElevatorSystem elevator) {
    //elevatorController = elevator.getPIDController();
    // Use addRequirements() here to declare subsystem dependencies.

    driveToPose = drivetrain.driveToPosePrecise(scorePose).get();
    addRequirements(elevator, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    driveToPose.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveToPose.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    driveToPose.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
