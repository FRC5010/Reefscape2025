// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import org.frc5010.common.motors.hardware.GenericTalonFXMotor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class TalonFXEncoder implements GenericEncoder {
  /** TalonFX motor */
  TalonFX motor;
    /** TalonFX simulation */
  protected TalonFXSimState talonFXSim;
  double metersPerRotation = 1;
  double metersPerSecPerRPM = 1;
  private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation

  public TalonFXEncoder(GenericTalonFXMotor motor) {
    this.motor = (TalonFX)motor.getMotor();
    talonFXSim = this.motor.getSimState();
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  public double rotationsToDistance(double position) {
    return position * metersPerRotation;
  }

  public double distanceToRotations(double position) {
    return position / metersPerRotation;
  }

  public double rotationsPerMinToVelocity(double velocity) {
    return velocity * metersPerSecPerRPM;
  }

  public double velocityToRotationsPerMin(double velocity) {
    return velocity / metersPerSecPerRPM;
  }

  @Override
  public double getPosition() {
    return rotationsToDistance(motor.getPosition().getValue().in(Rotations));
  }

  @Override
  public double getVelocity() {
    return rotationsPerMinToVelocity(motor.getVelocity().getValue().in(RPM));
  }

  public double getVoltage() {
    return talonFXSim.getMotorVoltage();
  }
  
  @Override
  public void reset() {
    setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    motor.setPosition(distanceToRotations(position));
    talonFXSim.setRawRotorPosition(distanceToRotations(position));
  }

  @Override
  public void setRate(double rate) {
    talonFXSim.setRotorVelocity(velocityToRotationsPerMin(rate) / 60.0);
  }

  @Override
  public void setPositionConversion(double conversion) {
    metersPerRotation = conversion;
  }

  @Override
  public void setVelocityConversion(double conversion) {
    metersPerSecPerRPM = conversion;
  }

  @Override
  public void setInverted(boolean inverted) {
    if(inverted) {
      talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
    } else {
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }
  }

  @Override
  public double getPositionConversion() {
    return metersPerRotation;
  }

  @Override
  public double getVelocityConversion() {
    return metersPerSecPerRPM;
  }
  
  @Override
  public void simulationUpdate(Optional<Double> position, Double velocity) {
    // set the supply voltage of the TalonFX
    if (position.isPresent()) {
      setPosition(position.get());
    }
    setRate(velocity);
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage() - talonFXSim.getSupplyCurrent() * kMotorResistance);
  }
}
