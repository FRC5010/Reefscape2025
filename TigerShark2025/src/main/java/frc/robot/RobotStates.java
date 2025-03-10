// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.frc5010.common.subsystems.NewLEDSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ElevatorSystem.Position;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.ShooterSystem.CoralState;

/** Add your docs here. */
public class RobotStates {

  private ShooterSystem shooter;
  private AlgaeArm algaeArm;
  private ElevatorSystem elevator;
  private ClimbSubsystem climb;

  private RobotState state;
  private CoralState coralState;
  private AlgaeState algaeState;
  private Supplier<Pose2d> robotPose;

  private Trigger empty, coralLoading, coralLoaded, algaeLoaded, scoringCoral, scoringProcessor, scoringBarge,
      descoringAlgae, climbing, poleAlignment;

  public enum RobotState {
    EMPTY,
    CORAL_LOADING,
    CORAL_LOADED,
    ALGAE_LOADED,
    SCORING_CORAL,
    SCORING_PROCESSOR,
    DESCORING_ALGAE,
    SCORING_BARGE,
    CLIMBING,
    POLE_ALIGNMENT
  }

  // TODO: Add pole positioning leds
  public RobotStates(ShooterSystem shooter, AlgaeArm algaeArm, ElevatorSystem elevator, ClimbSubsystem climb, NewLEDSubsystem leds, Supplier<Pose2d> robotPose) {
    this.shooter = shooter;
    this.algaeArm = algaeArm;
    this.elevator = elevator;
    this.climb = climb;
    this.robotPose = robotPose;

    coralState = shooter.getCoralState();
    algaeState = algaeArm.getAlgaeState();

    state = RobotState.EMPTY;

    empty = new Trigger(() -> coralState == CoralState.EMPTY && algaeState == AlgaeState.RETRACTED);
    coralLoading = new Trigger(() -> coralState == CoralState.ENTRY); // Change logic when beambreaks are back
    coralLoaded = new Trigger(() -> coralState == CoralState.FULLY_CAPTURED); // Change logic when beambreaks are back
    algaeLoaded = new Trigger(() -> algaeState == AlgaeState.DEPLOYED);
    scoringCoral = new Trigger(() -> shooter.getAverageMotorSpeed() > 0.01 && !elevator.atLoading());
    scoringProcessor = new Trigger(
        () -> elevator.isAtLocation(Position.PROCESSOR.position()) && algaeState != AlgaeState.RETRACTED);
    scoringBarge = new Trigger(
        () -> elevator.isAtLocation(Position.NET.position()) && algaeState != AlgaeState.RETRACTED);
    descoringAlgae = new Trigger(() -> elevator.getElevatorPosition().in(Meters) > Position.L1.position().in(Meters)
        && elevator.getElevatorPosition().in(Meters) < Position.L4.position().in(Meters)
        && (algaeState == AlgaeState.DEPLOYING || algaeState == AlgaeState.RETRACTING));
    climbing = new Trigger(() -> climb.isClimbing());
    // poleAlignment = new Trigger().and(coralLoaded);

    empty.and(climbing.negate()).onTrue(Commands.runOnce(() -> {
      state = RobotState.EMPTY;
      leds.setPattern(leds.getMaskedPattern(leds.getSolidPattern(Color.kWhite), 0.5, 0.5));
    }));
    coralLoading.onTrue(Commands.runOnce(() -> {
      state = RobotState.CORAL_LOADING;
      leds.setPattern(leds.getBlinkingPattern(leds.getSolidPattern(Color.kYellow), Seconds.of(0.1)));
    }));
    coralLoaded.and(scoringCoral.negate()).onTrue(Commands.runOnce(() -> {
      state = RobotState.CORAL_LOADED;
      leds.setPattern(leds.getSolidPattern(Color.kGreen));
    }));
    algaeLoaded.and(scoringProcessor.negate()).and(scoringBarge.negate()).and(descoringAlgae.negate())
        .onTrue(Commands.runOnce(() -> {
          state = RobotState.ALGAE_LOADED;
          leds.setPattern(leds.getSolidPattern(Color.kBlue));
        }));
    scoringCoral.onTrue(Commands.runOnce(() -> {
      state = RobotState.SCORING_CORAL;
      leds.setPattern(leds.getBlinkingPattern(leds.getSolidPattern(Color.kRed), Seconds.of(0.1)));
    }));
    scoringProcessor.onTrue(Commands.runOnce(() -> {
      state = RobotState.SCORING_PROCESSOR;
      leds.setPattern(leds.getBlinkingPattern(leds.getSolidPattern(Color.kViolet), Seconds.of(0.1)));
    }));
    scoringBarge.onTrue(Commands.runOnce(() -> {
      state = RobotState.SCORING_BARGE;
      leds.setPattern(leds.getBlinkingPattern(leds.getRainbowPattern(3.0), Seconds.of(0.1)));
    }));
    descoringAlgae.onTrue(Commands.runOnce(() -> {
      state = RobotState.DESCORING_ALGAE;
      leds.setPattern(leds.getBlinkingPattern(leds.getSolidPattern(Color.kOrange), Seconds.of(0.1)));
    }));
    climbing.onTrue(Commands.runOnce(() -> {
      state = RobotState.CLIMBING;
      leds.setPattern(leds.getMaskedPattern(leds.getSolidPattern(Color.kViolet), 0.5, 1));
    })); // TODO: Eventually add progress bar
  }

  public RobotState getRobotState() {
    return state;
  }
}
