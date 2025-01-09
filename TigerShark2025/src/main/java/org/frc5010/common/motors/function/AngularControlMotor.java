// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Provides PID control for angular motors */
public class AngularControlMotor extends GenericControlledMotor {
  protected MechanismLigament2d simulatedArm;
  protected MechanismLigament2d setpoint;
  protected MechanismRoot2d root;
  protected SingleJointedArmSim simMotor;
  protected SimulatedEncoder simEncoder;
  protected GenericEncoder encoder;
  protected Distance armLength = Meters.of(0.0);
  protected Angle minAngle = Degrees.of(0.0);
  protected Angle maxAngle = Degrees.of(0.0);
  protected Angle startingAngle = Degrees.of(0.0);
  protected final String K_G = "kG";
  protected DisplayDouble kG;

  public AngularControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName, tab);
    kG = new DisplayDouble(0.0, K_G, visualName);
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

    simMotor =
        new SingleJointedArmSim(
            _motor.getMotorSimulationType(),
            gearing,
            SingleJointedArmSim.estimateMOI(armLength.in(Meters), mass),
            armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            simulateGravity,
            startingAngle.in(Radians));
    simEncoder =
        new SimulatedEncoder(
            MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
    simEncoder.setInverted(inverted);
    simEncoder.setPosition(startingAngle.in(Degrees));
    simEncoder.setPositionConversion(conversion);
    position.setValue(startingAngle.in(Degrees));
    return this;
  }

  @Override
  public AngularControlMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root =
        visualizer.getRoot(
            _visualName,
            getSimX(Meters.of(robotToMotor.getX())),
            getSimY(Meters.of(robotToMotor.getZ())));
    simulatedArm =
        new MechanismLigament2d(
            _visualName + "-arm",
            armLength.in(Meters),
            startingAngle.in(Degrees),
            5,
            new Color8Bit(MotorFactory.getNextVisualColor()));
    setpoint =
        new MechanismLigament2d(
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
    super.setReference(reference);
    if (!RobotBase.isReal()) {
      effort.setVoltage(calculateControlEffort(simEncoder.getPosition()), Volts);
      _motor.set(effort.getVoltage().in(Volts) / RobotController.getBatteryVoltage());
    } else {
      pid.setF(getFeedForward().in(Volts) / RobotController.getBatteryVoltage());
    }
  }

  public double getPivotPosition() {
    if (RobotBase.isReal()) {
      return encoder.getPosition() > 180 ? encoder.getPosition() - 360 : encoder.getPosition();
    } else {
      return simEncoder.getPosition();
    }
  }

  @Override
  public Voltage getFeedForward() {
    ArmFeedforward pivotFeedforward =
        new ArmFeedforward(
            getMotorFeedFwd().getkS(),
            kG.getValue(),
            getMotorFeedFwd().getkV(),
            getMotorFeedFwd().getkA());
    Voltage ff = Volts.of(
        pivotFeedforward.calculate(
            Degrees.of(getPivotPosition()).in(Radians), 0.0));
    feedForward.setValue(ff.in(Volts));
    return ff;
  }

  @Override
  public void draw() {
    double currentPosition = 0;
    if (RobotBase.isReal()) {
      currentPosition = encoder.getPosition();
    } else {
      currentPosition = simEncoder.getPosition();
    }
    position.setValue(currentPosition);
    reference.setValue(getReference());
    simulatedArm.setAngle(currentPosition);
    setpoint.setAngle(getReference());
  }

  @Override
  public void simulationUpdate() {
    simMotor.setInput(_motor.get() * RobotController.getBatteryVoltage());
    simMotor.update(0.020);
    simEncoder.setPosition(Units.radiansToDegrees(simMotor.getAngleRads()));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simMotor.getCurrentDrawAmps()));
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
}
