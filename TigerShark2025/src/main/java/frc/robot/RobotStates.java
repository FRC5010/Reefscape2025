// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.ShooterSystem.CoralState;

/** Add your docs here. */
public class RobotStates {

  private ShooterSystem shooter;
  private AlgaeArm algaeArm;
  private ElevatorSystem elevator;
  private ClimbSubsystem climb;

  private RobotState state;
  private Supplier<CoralState> coralState;
  private Supplier<AlgaeState> algaeState;
  private Supplier<Pose2d> robotPose;



  public enum RobotState {
    IDLE,
    CORAL_LOADING,
    SCORING_CORAL,
    ALIGNING_ON_POLE_LOADED,
    ALIGNING_ON_POLE_EMPTY,
    SCORING_PROCESSOR,
    DESCORING_ALGAE,
    SCORING_BARGE,
    CLIMBING
  }

  // TODO: Add pole positioning leds
  public RobotStates(ShooterSystem shooter, AlgaeArm algaeArm, ElevatorSystem elevator, ClimbSubsystem climb, Supplier<Pose2d> robotPose) {
    this.shooter = shooter;
    this.algaeArm = algaeArm;
    this.elevator = elevator;
    this.climb = climb;
    this.robotPose = robotPose;

    coralState = () -> shooter.getCoralState();
    algaeState = () -> algaeArm.getAlgaeState();

    state = RobotState.IDLE;


  }

  public void setCurrentState(RobotState state) {
    this.state = state;
  }

  public RobotState getRobotState() {
    return state;
  }
}
