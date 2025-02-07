// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.sensors.Beambreak;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class ShooterSystem extends GenericSubsystem {
  Beambreak alignmentBeambreak;
  Beambreak entryBeambreak;

  protected VelocityControlMotor shooterLeft;
  protected VelocityControlMotor shooterRight;
  private Trigger entryBroken, alignmentBroken, empty, entry, loading, aligned;
  private CoralState coralState;

  public enum CoralState{
    EMPTY,
    ENTRY,
    LOADING,
    ALIGNED
  }

  /** Creates a new Shooter. */
  public ShooterSystem(Mechanism2d mechanismSimulation) {
    coralState = CoralState.EMPTY;

    alignmentBeambreak = new Beambreak(0);
    entryBeambreak = new Beambreak(1);

    entryBroken = new Trigger(entryBeambreak.isBrokenSupplier());
    entryBroken.and(() -> coralState == CoralState.EMPTY).onTrue(setCoralState(CoralState.ENTRY));
    alignmentBroken = new Trigger(alignmentBeambreak.isBrokenSupplier());
    alignmentBroken.and(() -> coralState == CoralState.ENTRY).onTrue(setCoralState(CoralState.LOADING));
    alignmentBroken.and(() -> coralState == CoralState.LOADING).onFalse(setCoralState(CoralState.ALIGNED));
    alignmentBroken.and(() -> coralState == CoralState.ALIGNED).onFalse(setCoralState(CoralState.EMPTY));

    empty = new Trigger(() -> coralState == CoralState.EMPTY);
    entry = new Trigger(() -> coralState == CoralState.ENTRY);
    loading = new Trigger(() -> coralState == CoralState.LOADING);
    aligned = new Trigger(() -> coralState == CoralState.ALIGNED);

    shooterLeft = new VelocityControlMotor(MotorFactory.Spark(11, Motor.Neo), "shooterLeft", displayValues);
    shooterRight = new VelocityControlMotor(MotorFactory.TalonFX(12, Motor.KrakenX60), "shooterRight",
            displayValues);
    shooterLeft.setupSimulatedMotor(1, 10);
    shooterRight.setupSimulatedMotor(1, 10);
    shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(
            new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters), Inches.of(16.25).in(Meters)),
            new Rotation3d()));
    shooterRight.setVisualizer(mechanismSimulation,
            new Pose3d(new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters),
                  Inches.of(6.25).in(Meters)), new Rotation3d()));

    empty.onTrue(runMotors(0.0));
    entry.whileTrue(runMotors(0.1));
    loading.whileTrue(runMotors(-0.1));
  }

  public void shooterLeftSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void shooterRightSpeed(double speed) {
      shooterRight.setReference(speed * shooterRight.getMaxRPM().in(RPM));
  }

  public Command runMotors(double speed) {
    return Commands.run(() -> {
      shooterLeftSpeed(speed);
      shooterRightSpeed(speed);
    }, this);
  }

  public Command setCoralState(CoralState state) {
    return Commands.runOnce(() -> coralState = state);
  }

  public Command intakeCoral() {
    return runMotors(0.1).until(empty.negate());
  }

  @Override
  public void periodic() {
      shooterLeft.draw();
      shooterRight.draw();
  }

  @Override
  public void simulationPeriodic() {
      shooterLeft.simulationUpdate();
      shooterRight.simulationUpdate();
  }
}
