// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class VelocityControlMotor extends GenericControlledMotor {
  protected MechanismLigament2d speedometer;
  protected MechanismLigament2d setpoint;
  protected MechanismRoot2d root;
  protected FlywheelSim simMotor;
  protected SimulatedEncoder simEncoder;

  public VelocityControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper display) {
    super(motor, visualName, display);
    pid.setControlType(PIDControlType.VELOCITY);
  }

  public VelocityControlMotor setupSimulatedMotor(double gearing, double jKgMetersSquared) {
    simMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(_motor.getMotorSimulationType(), jKgMetersSquared, gearing),
        _motor.getMotorSimulationType());
    simEncoder =
        new SimulatedEncoder(
            MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
    return this;
  }

  @Override
  public VelocityControlMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root =
        visualizer.getRoot(
            _visualName,
            getSimX(Meters.of(robotToMotor.getX())),
            getSimY(Meters.of(robotToMotor.getZ())));
    speedometer =
        new MechanismLigament2d(
            _visualName + "-velocity", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    setpoint =
        new MechanismLigament2d(
            _visualName + "-setpoint", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    root.append(speedometer);
    root.append(setpoint);
    return this;
  }

  @Override
  public void draw() {
    double currentVelocity = 0;
    if (RobotBase.isReal()) {
      currentVelocity = encoder.getVelocity();
    } else {
      currentVelocity = simEncoder.getVelocity();
    }
    velocity.setValue(currentVelocity);
    reference.setValue(getReference());
    speedometer.setAngle(270 - currentVelocity / _motor.getMaxRPM().in(Rotations.per(Minute)) * 180);
    setpoint.setAngle(270 - getReference() / _motor.getMaxRPM().in(Rotations.per(Minute)) * 180);
  }

  @Override
  public void simulationUpdate() {
    effort.setVoltage(calculateControlEffort(simEncoder.getVelocity()), Volts);
    simMotor.setInput(effort.getVoltage().in(Volts));
    simMotor.update(0.020);
    simEncoder.setRate(simMotor.getAngularVelocityRPM());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simMotor.getCurrentDrawAmps()));
  }

  @Override
  public Command getSysIdCommand(SubsystemBase subsystemBase) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(_motor, encoder, _visualName, subsystemBase), 5, 3, 3);
  }
}
