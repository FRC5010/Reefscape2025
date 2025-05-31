// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.frc5010.common.telemetry.DisplayVoltage;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class PercentControlMotor extends GenericFunctionalMotor {
  protected LoggedMechanismLigament2d speedometer;
  protected LoggedMechanismRoot2d root;
  protected FlywheelSim simMotor;
  protected SimulatedEncoder simEncoder;
  protected DisplayDouble speed;
  protected DisplayVoltage effort;
  protected DisplayDouble simRPM;

  public PercentControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName);
    setDisplayValuesHelper(tab);
  }

  @Override
  public void initiateDisplayValues() {
    speed = _displayValuesHelper.makeDisplayDouble("Speed");
    effort = _displayValuesHelper.makeDisplayVoltage("Effort");
    simRPM = _displayValuesHelper.makeDisplayDouble("Sim RPM");
  }

  public PercentControlMotor(MotorController5010 motor, double slewRate) {
    super(motor, slewRate);
  }

  public PercentControlMotor setupSimulatedMotor(double gearing, double momentOfInertiaKgMetersSq) {
    simMotor = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(12.0/_motor.getMaxRPM().in(RotationsPerSecond),0.001),
        //LinearSystemId.createFlywheelSystem(_motor.getMotorSimulationType(), momentOfInertiaKgMetersSq, gearing),
        _motor.getMotorSimulationType());
    simEncoder = new SimulatedEncoder(
        MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
    return this;
  }

  @Override
  public PercentControlMotor setVisualizer(LoggedMechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root = visualizer.getRoot(
        _visualName,
        getSimX(Meters.of(robotToMotor.getX())),
        getSimY(Meters.of(robotToMotor.getZ())));
    speedometer = new LoggedMechanismLigament2d(
        _visualName + "-speed", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    root.append(speedometer);
    return this;
  }

  @Override
  public void periodicUpdate() {
    speed.setValue(_motor.getMotorEncoder().getVelocity());
    speedometer.setAngle(270 - _motor.get() * 180);
  }

  @Override
  public void simulationUpdate() {
    effort.setVoltage(_motor.getVoltage(), Volts);
    simMotor.setInput(effort.getVoltageInVolts());
    simMotor.update(0.020);
    simRPM.setValue(simMotor.getAngularVelocityRPM());
    _motor.simulationUpdate(Optional.empty(), simRPM.getValue());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simMotor.getCurrentDrawAmps()));
  }
}
