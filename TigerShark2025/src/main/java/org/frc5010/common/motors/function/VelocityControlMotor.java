// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class VelocityControlMotor extends GenericControlledMotor {
  protected LoggedMechanismLigament2d speedometer;
  protected LoggedMechanismLigament2d setpoint;
  protected LoggedMechanismRoot2d root;
  protected FlywheelSim flyWheelSim;

  public VelocityControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper display) {
    super(motor, visualName, display);
    controller.setControlType(PIDControlType.VELOCITY);
  }

  public VelocityControlMotor setupSimulatedMotor(double gearing, double jKgMetersSquared) {
    flyWheelSim = new FlywheelSim(
      LinearSystemId.identifyVelocitySystem(12.0/_motor.getMaxRPM().in(RotationsPerSecond),0.001),
        //LinearSystemId.createFlywheelSystem(_motor.getMotorSimulationType(), jKgMetersSquared, gearing),
        _motor.getMotorSimulationType());
    return this;
  }

  @Override
  public VelocityControlMotor setVisualizer(LoggedMechanism2d visualizer, Pose3d robotToMotor) {
    super.setVisualizer(visualizer, robotToMotor);

    root =
        visualizer.getRoot(
            _visualName,
            getSimX(Meters.of(robotToMotor.getX())),
            getSimY(Meters.of(robotToMotor.getZ())));
    speedometer =
        new LoggedMechanismLigament2d(
            _visualName + "-velocity", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    setpoint =
        new LoggedMechanismLigament2d(
            _visualName + "-setpoint", 0.1, 0, 5, new Color8Bit(MotorFactory.getNextVisualColor()));
    root.append(speedometer);
    root.append(setpoint);
    return this;
  }

  @Override
  public void periodicUpdate() {
    double currentVelocity = 0;
    currentVelocity = encoder.getVelocity();
    velocity.setValue(currentVelocity);
    speedometer.setAngle(270 - currentVelocity / _motor.getMaxRPM().in(RPM) * 180);
    setpoint.setAngle(270 - getReference() / _motor.getMaxRPM().in(RPM) * 180);
  }

  @Override
  public void simulationUpdate() {
    flyWheelSim.setInput(_motor.getVoltage());
    outputEffort.setVoltage(_motor.getVoltage(), Volts);
    flyWheelSim.update(0.020);
    
    _motor.simulationUpdate(Optional.empty(), flyWheelSim.getAngularVelocityRPM());
        
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flyWheelSim.getCurrentDrawAmps()));
  }

  @Override
  public Command getSysIdCommand(SubsystemBase subsystemBase) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(_motor, encoder, _visualName, subsystemBase), 5, 3, 3);
  }

  @Override
  public double getEncoderFeedback() {
    return encoder.getVelocity();
  }
}
