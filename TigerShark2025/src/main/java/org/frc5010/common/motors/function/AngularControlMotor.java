// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Provides PID control for angular motors */
public class AngularControlMotor extends GenericControlledMotor {
  protected LoggedMechanismLigament2d simulatedArm;
  protected LoggedMechanismLigament2d setpoint;
  protected LoggedMechanismRoot2d root;
  protected SingleJointedArmSim simMechanism;
  protected GenericEncoder encoder;
  protected Distance armLength = Meters.of(0.0);
  protected Angle minAngle = Degrees.of(0.0);
  protected Angle maxAngle = Degrees.of(0.0);
  protected Angle startingAngle = Degrees.of(0.0);
  protected final String K_G = "kG";
  protected DisplayDouble kG;
  ArmFeedforward pivotFeedforward;

  public AngularControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName, tab);
    kG = _displayValuesHelper.makeConfigDouble(K_G);
    encoder = motor.getMotorEncoder();
    setControlType(PIDControlType.POSITION);
  }

  public AngularControlMotor setupSimulatedMotor(
      double gearing,
      double mass,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      boolean simulateGravity,
      double kG,
      Angle startingAngle,
      boolean inverted,
      double conversion) {
    this.armLength = armLength;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.startingAngle = startingAngle;
    this.kG.setValue(kG);

    simMechanism = new SingleJointedArmSim(
        _motor.getMotorSimulationType(),
        gearing,
        SingleJointedArmSim.estimateMOI(armLength.in(Meters), mass),
        armLength.in(Meters),
        minAngle.in(Radians),
        maxAngle.in(Radians),
        simulateGravity,
        startingAngle.in(Radians));

    encoder.setInverted(inverted);
    encoder.setPosition(startingAngle.in(Degrees));
    encoder.setPositionConversion(conversion / gearing);
    position.setValue(startingAngle.in(Degrees));
    return this;
  }

  @Override
  public AngularControlMotor setVisualizer(LoggedMechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root = visualizer.getRoot(
        _visualName,
        getSimX(Meters.of(robotToMotor.getX())),
        getSimY(Meters.of(robotToMotor.getZ())));
    simulatedArm = new LoggedMechanismLigament2d(
        _visualName + "-arm",
        armLength.in(Meters),
        startingAngle.in(Degrees),
        5,
        new Color8Bit(MotorFactory.getNextVisualColor()));
    setpoint = new LoggedMechanismLigament2d(
        _visualName + "-setpoint",
        armLength.in(Meters),
        startingAngle.in(Degrees),
        5,
        new Color8Bit(MotorFactory.getNextVisualColor()));
    root.append(simulatedArm);
    root.append(setpoint);
    return this;
  }

  @Override
  public void setReference(double reference) {
    setReference(reference, controller.getControlType(),
        getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage());
  }

  public void updateReference() {
    if (PIDControlType.NONE != controller.getControlType()) {
      controller.setReference(reference.getValue(), getControlType(),
          getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage());
    }
  }

  public double getPivotPosition() {
    if (RobotBase.isReal()) {
      return encoder.getPosition() > 180 ? encoder.getPosition() - 360 : encoder.getPosition();
    } else {
      return Units.radiansToDegrees(simMechanism.getAngleRads());
    }
  }

  @Override
  public Voltage getFeedForward(double velocity) {
    if (null == pivotFeedforward || _displayValuesHelper.getLoggingLevel() == LogLevel.CONFIG) {
      pivotFeedforward = new ArmFeedforward(
          getMotorFeedFwd().getkS(),
          kG.getValue(),
          getMotorFeedFwd().getkV(),
          getMotorFeedFwd().getkA());
    }
    Voltage ff = Volts.of(
        pivotFeedforward.calculate(
            Degrees.of(getPivotPosition()).in(Radians), 0.0));
    feedForward.setValue(ff.in(Volts));
    return ff;
  }

  @Override
  public void periodicUpdate() {
    updateReference();
    double currentPosition = getPivotPosition();
    position.setValue(currentPosition);
    velocity.setValue(encoder.getVelocity());
    simulatedArm.setAngle(currentPosition);
    setpoint.setAngle(getReference());
  }

  @Override
  public void simulationUpdate() {
    simMechanism.setInput(_motor.getVoltage());
    outputEffort.setVoltage(_motor.getVoltage(), Volts);
    simMechanism.update(0.020);
    _motor.simulationUpdate(Optional.of(simMechanism.getAngleRads()),
        simMechanism.getVelocityRadPerSec());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simMechanism.getCurrentDrawAmps()));
  }

  public boolean isAtMaximum() {
    return encoder.getPosition() >= maxAngle.in(Degrees);
  }

  public boolean isAtMinimum() {
    return encoder.getPosition() <= minAngle.in(Degrees);
  }

  public boolean isAtStartingAngle() {
    return encoder.getPosition() == startingAngle.in(Degrees);
  }

  public boolean isAtTarget() {
    return Math.abs(getReference() - getPivotPosition()) < tolerance.getValue();
  }

  public void setEncoder(GenericEncoder encoder) {
    this.encoder = encoder;
  }

  public Command getSysIdCommand(SubsystemBase subsystemBase) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(_motor, encoder, "Angular Motor", subsystemBase),
        5,
        3,
        3);
  }

  @Override
  public double getEncoderFeedback() {
    return encoder.getPosition();
  }
}
