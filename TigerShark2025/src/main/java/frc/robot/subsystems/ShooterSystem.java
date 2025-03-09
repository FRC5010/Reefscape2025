// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.sensors.Beambreak;
import org.frc5010.common.sensors.ValueSwitch;
import org.frc5010.common.telemetry.DisplayBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;


public class ShooterSystem extends GenericSubsystem {
  Beambreak alignmentBeambreak;
  Beambreak entryBeambreak;

  protected ValueSwitch currentSwitch;
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

  public static class Config {
    private final Current INTAKE_CURRENT_THRESHOLD = Amps.of(3);
    int alignmentBeambreakCanID = 8;
    int entryBeambreakCanID = 7;
    int shooterLeftCanID = 11;
    int shooterRightCanID = 12;
    boolean entryBeambreakDisplay = false;
    boolean alignmentBeambreakDisplay = false;
    double currentSwitchTriggerThreshold = 1.0;
    boolean shooterLeftInverted = false;
    boolean shooterRightInverted = true;
    double shooterLeftGearing = 1.0;
    double shooterRightGearing = 1.0;
    double shooterLeftJKGMetersSquared = 10.0;
    double shooterRightJKGMetersSquared = 10.0;
    Distance shooterLeftX = Inches.of(7.15);
    Distance shooterLeftY = Inches.of(2.875);
    Distance shooterLeftZ = Inches.of(16.25);
    Distance shooterRightX = Inches.of(7.15);
    Distance shooterRightY = Inches.of(2.875);
    Distance shooterRightZ = Inches.of(16.25);
    double runMotorsSpeedMultiplier = 0.5;
  }

  private Config config = new Config();

  /** Creates a new Shooter. */
  public ShooterSystem(Mechanism2d mechanismSimulation, Config config) {
    if (config != null) this.config = config;
    
    alignmentBeambreak = new Beambreak(config.alignmentBeambreakCanID);
    entryBeambreak = new Beambreak(config.entryBeambreakCanID);

    entryBeamBreakDisplay = displayValues.makeDisplayBoolean("Entry Beam Break");
    alignmentBeamBreakDisplay = displayValues.makeDisplayBoolean("Alignment Beam Break");
    
    entryBeamBreakDisplay.setValue(config.entryBeambreakDisplay);
    alignmentBeamBreakDisplay.setValue(config.alignmentBeambreakDisplay);

    entryBroken = new Trigger(entryBeambreak.isBrokenSupplier());
    alignmentBroken = new Trigger(alignmentBeambreak.isBrokenSupplier());

    coralState = alignmentBeambreak.isBroken() ? CoralState.FULLY_CAPTURED : CoralState.EMPTY;

    isEmpty = new Trigger(() -> coralState == CoralState.EMPTY);
    // isEntryActive = new Trigger(() -> coralState == CoralState.ENTRY);
    isCoralFullyCaptured = new Trigger(() -> coralState == CoralState.FULLY_CAPTURED);
    isAligned = new Trigger(() -> coralState == CoralState.ALIGNED);

    
    setupMotors(mechanismSimulation);

    currentSwitch = new ValueSwitch(config.INTAKE_CURRENT_THRESHOLD.in(Amps), shooterLeft::getOutputCurrent, config.currentSwitchTriggerThreshold);
    
    setupStateMachine();
    // isEntryActive.whileTrue(captureCoral());
    // isCoralFullyCaptured.whileTrue(alignCoral());
   
  }

  private void setupMotors(Mechanism2d mechanismSimulation) {
    shooterLeft = new VelocityControlMotor(MotorFactory.Thrifty(config.shooterLeftCanID, Motor.Neo), "shooterLeft", displayValues);
    shooterRight = new VelocityControlMotor(MotorFactory.Thrifty(config.shooterRightCanID, Motor.Neo), "shooterRight",
            displayValues);
    shooterLeft.invert(config.shooterLeftInverted);
    shooterRight.invert(config.shooterRightInverted);


    shooterLeft.setupSimulatedMotor(config.shooterLeftGearing, config.shooterLeftJKGMetersSquared);
    shooterRight.setupSimulatedMotor(config.shooterRightGearing, config.shooterRightJKGMetersSquared);
    shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(
            new Translation3d(config.shooterLeftX.in(Meters), config.shooterLeftY.in(Meters), config.shooterLeftZ.in(Meters)),
            new Rotation3d()));
    shooterRight.setVisualizer(mechanismSimulation,
            new Pose3d(new Translation3d(config.shooterRightX.in(Meters), config.shooterRightY.in(Meters),
                  config.shooterRightZ.in(Meters)), new Rotation3d()));
  }

  private void setupStateMachine() {
    Trigger coralOutOfShooter = entryBroken.negate().and(alignmentBroken.negate());

    // Empty
    alignmentBroken.and(isEmpty).onTrue(setCoralState(CoralState.FULLY_CAPTURED));
    alignmentBroken.negate().onTrue(setCoralState(CoralState.EMPTY));
    // Entry
    // alignmentBroken.and(currentSwitch.getTrigger()).onTrue(setCoralState(CoralState.FULLY_CAPTURED)); // Entry -> Fully Captured
    // coralOutOfShooter.and(isEntryActive).onTrue(setCoralState(CoralState.EMPTY)); // Entry -> Empty
    // // Fully Captured
    // alignmentBroken.and(isCoralFullyCaptured).onFalse(setCoralState(CoralState.ALIGNED)); // Fully Captured -> Aligned
    // coralOutOfShooter.and(isCoralFullyCaptured).onTrue(setCoralState(CoralState.EMPTY)); // Fully Captured -> Empty
    // // Aligned
    // alignmentBroken.and(isAligned).onFalse(setCoralState(CoralState.FULLY_CAPTURED)); // Aligned -> Fully Captured
    // coralOutOfShooter.and(isAligned).onTrue(setCoralState(CoralState.EMPTY)); // Aligned -> Empty
  }

  public void shooterLeftSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void shooterRightSpeed(double speed) {
      shooterRight.set(speed);
  }

  public Command runMotors(DoubleSupplier speed) {
    return Commands.run(() -> {
      shooterLeftSpeed(speed.getAsDouble() * config.runMotorsSpeedMultiplier);
      shooterRightSpeed(speed.getAsDouble() * config.runMotorsSpeedMultiplier);
    }, this);
  }

  public Command runMotors(DoubleSupplier speedLeft, DoubleSupplier speedRight) {
    return Commands.run(() -> {
      shooterLeftSpeed(speedLeft.getAsDouble());
      shooterRightSpeed(speedRight.getAsDouble());
    }, this);
  }

  public Command shootL1() {
    return Commands.run(()->{
      shooterLeftSpeed(1.0);
      shooterRightSpeed(0.7);
  }, this).until(isEmpty);
  }

  public Command getShootCommand(ScoringLevel level) {
    if (ScoringLevel.L1 == level) {
      return shootL1();
    } 
    return runMotors(() -> 1.0).until(isEmpty());
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

  public CoralState getCoralState() {
    return coralState;
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

  public double getAverageMotorSpeed() {
    return (Math.abs(shooterLeft.get()) + Math.abs(shooterRight.get())) / 2;
  }

  @Override
  public void periodic() {
      shooterLeft.draw();
      shooterRight.draw();
      entryBeamBreakDisplay.setValue(entryBroken.getAsBoolean());
      alignmentBeamBreakDisplay.setValue(alignmentBroken.getAsBoolean());
      
      coralState = alignmentBeambreak.isBroken() ? CoralState.FULLY_CAPTURED : CoralState.EMPTY;

      SmartDashboard.putString("Coral State", coralState.name());
      SmartDashboard.putNumber("Shooter Left Current", shooterLeft.getOutputCurrent());
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
