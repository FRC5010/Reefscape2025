// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.sensors.Beambreak;
import org.frc5010.common.telemetry.DisplayBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class ShooterSystem extends GenericSubsystem {
  Beambreak alignmentBeambreak;
  Beambreak entryBeambreak;

  protected VelocityControlMotor shooterLeft;
  protected VelocityControlMotor shooterRight;
  private Trigger entryBroken, alignmentBroken;
  private Trigger isEmpty, isEntryActive, isCoralFullyCaptured, isAligned;
  private CoralState coralState;
  private DisplayBoolean entryBeamBreakDisplay, alignmentBeamBreakDisplay;

  public enum CoralState{
    EMPTY,
    ENTRY,
    FULLY_CAPTURED,
    ALIGNED
  }

  /** Creates a new Shooter. */
  public ShooterSystem(Mechanism2d mechanismSimulation) {
    

    alignmentBeambreak = new Beambreak(1);
    entryBeambreak = new Beambreak(0);

    entryBeamBreakDisplay = displayValues.makeDisplayBoolean("Entry Beam Break");
    alignmentBeamBreakDisplay = displayValues.makeDisplayBoolean("Alignment Beam Break");
    
    entryBeamBreakDisplay.setValue(false);
    alignmentBeamBreakDisplay.setValue(false);

    entryBroken = new Trigger(entryBeambreak.isBrokenSupplier());
    alignmentBroken = new Trigger(alignmentBeambreak.isBrokenSupplier());

    coralState = entryBeambreak.isBroken() ? CoralState.FULLY_CAPTURED : CoralState.EMPTY;

    isEmpty = new Trigger(() -> coralState == CoralState.EMPTY);
    isEntryActive = new Trigger(() -> coralState == CoralState.ENTRY);
    isCoralFullyCaptured = new Trigger(() -> coralState == CoralState.FULLY_CAPTURED);
    isAligned = new Trigger(() -> coralState == CoralState.ALIGNED);

    setupStateMachine();
    setupMotors(mechanismSimulation);
   
    // isEntryActive.whileTrue(captureCoral());
    // isCoralFullyCaptured.whileTrue(alignCoral());
   
  }

  private void setupMotors(Mechanism2d mechanismSimulation) {
    shooterLeft = new VelocityControlMotor(MotorFactory.Thrifty(11, Motor.Neo), "shooterLeft", displayValues);
    shooterRight = new VelocityControlMotor(MotorFactory.Thrifty(12, Motor.Neo), "shooterRight",
            displayValues);
    shooterLeft.invert(false);
    shooterRight.invert(true);


    shooterLeft.setupSimulatedMotor(1, 10);
    shooterRight.setupSimulatedMotor(1, 10);
    shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(
            new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters), Inches.of(16.25).in(Meters)),
            new Rotation3d()));
    shooterRight.setVisualizer(mechanismSimulation,
            new Pose3d(new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters),
                  Inches.of(6.25).in(Meters)), new Rotation3d()));
  }

  private void setupStateMachine() {
    Trigger coralOutOfShooter = entryBroken.negate().and(alignmentBroken.negate());

    // Empty
    entryBroken.and(isEmpty).onTrue(setCoralState(CoralState.ENTRY));
    // Entry
    alignmentBroken.and(isEntryActive).onTrue(setCoralState(CoralState.FULLY_CAPTURED)); // Entry -> Fully Captured
    coralOutOfShooter.and(isEntryActive).onTrue(setCoralState(CoralState.EMPTY)); // Entry -> Empty
    // Fully Captured
    alignmentBroken.and(isCoralFullyCaptured).onFalse(setCoralState(CoralState.ALIGNED)); // Fully Captured -> Aligned
    coralOutOfShooter.and(isCoralFullyCaptured).onTrue(setCoralState(CoralState.EMPTY)); // Fully Captured -> Empty
    // Aligned
    alignmentBroken.and(isAligned).onFalse(setCoralState(CoralState.FULLY_CAPTURED)); // Aligned -> Fully Captured
    coralOutOfShooter.and(isAligned).onTrue(setCoralState(CoralState.EMPTY)); // Aligned -> Empty
  }

  public void shooterLeftSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void shooterRightSpeed(double speed) {
      shooterRight.set(speed);
  }

  public Command runMotors(DoubleSupplier speed) {
    return Commands.run(() -> {
      shooterLeftSpeed(speed.getAsDouble() * 0.5);
      shooterRightSpeed(speed.getAsDouble() * 0.5);
    }, this);
  }

  public Command captureCoral() {
    return runMotors(() -> 0.1);
  }

  public Command alignCoral() {
    return runMotors(() -> -0.1).until(isAligned);
  }

  public Command setCoralState(CoralState state) {
    return Commands.runOnce(() -> coralState = state);
  }

  public Command intakeCoral() {
    return runMotors(() -> 0.1).until(isEmpty.negate());
  }

  public Trigger isEmpty() {
    return isEmpty;
  }

  public Trigger isFullyCaptured() {
    return isCoralFullyCaptured;
  }

  @Override
  public void periodic() {
      shooterLeft.draw();
      shooterRight.draw();
      entryBeamBreakDisplay.setValue(entryBroken.getAsBoolean());
      alignmentBeamBreakDisplay.setValue(alignmentBroken.getAsBoolean());

      SmartDashboard.putString("Coral State", coralState.name());
  }

  @Override
  public void simulationPeriodic() {
      shooterLeft.simulationUpdate();
      shooterRight.simulationUpdate();
  }

  public Command getSysIdCommand() {
    return shooterLeft.getSysIdCommand(this).andThen(shooterRight.getSysIdCommand(this));
  }
}
