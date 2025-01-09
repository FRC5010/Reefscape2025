// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class PercentControlMotor extends GenericFunctionalMotor {
  protected MechanismLigament2d speedometer;
  protected MechanismRoot2d root;
  protected FlywheelSim simMotor;
  protected SimulatedEncoder simEncoder;

  public PercentControlMotor(MotorController5010 motor, String visualName) {
    super(motor, visualName);
  }

  public PercentControlMotor(MotorController5010 motor, double slewRate) {
    super(motor, slewRate);
  }

  public PercentControlMotor setupSimulatedMotor(double gearing, double momentOfInertiaKgMetersSq) {
    simMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(_motor.getMotorSimulationType(), momentOfInertiaKgMetersSq, gearing),
        _motor.getMotorSimulationType());
    simEncoder = new SimulatedEncoder(
        MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
    return this;
  }

  @Override
  public PercentControlMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root = visualizer.getRoot(
        _visualName,
        getSimX(Meters.of(robotToMotor.getX())),
        getSimY(Meters.of(robotToMotor.getZ())));
    speedometer = new MechanismLigament2d(
        _visualName + "-speed", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    root.append(speedometer);
    return this;
  }

  @Override
  public void draw() {
    speedometer.setAngle(270 - _motor.get() * 180);
  }

  @Override
  public void simulationUpdate() {
    simMotor.setInput(_motor.get() * RobotController.getBatteryVoltage());
    simMotor.update(0.020);
    simEncoder.setRate(simMotor.getAngularVelocityRPM());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simMotor.getCurrentDrawAmps()));
  }
}
