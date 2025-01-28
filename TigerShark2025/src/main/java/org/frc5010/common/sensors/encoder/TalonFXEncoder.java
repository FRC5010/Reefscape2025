// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import java.util.Optional;

import org.frc5010.common.motors.hardware.GenericTalonFXMotor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class TalonFXEncoder implements GenericEncoder {
  /** TalonFX motor */
  TalonFX motor;
    /** TalonFX simulation */
  protected TalonFXSimState talonFXSim;
  double positionConversion = 1;
  double velocityConversion = 1;

  public TalonFXEncoder(GenericTalonFXMotor motor) {
    this.motor = (TalonFX)motor.getMotor();
    talonFXSim = this.motor.getSimState();
  }

  private double nativeToActualPosition(double position) {
    return position * positionConversion;
  }

  private double actualToNativePosition(double position) {
    return position / positionConversion;
  }

  private double nativeToActualVelocity(double velocity) {
    return velocity * velocityConversion;
  }

  private double actualToNativeVelocity(double velocity) {
    return velocity / velocityConversion;
  }

  @Override
  public double getPosition() {
    return nativeToActualPosition(motor.getPosition().getValueAsDouble());
  }

  @Override
  public double getVelocity() {
    return nativeToActualVelocity(motor.getVelocity().getValueAsDouble());
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
    motor.setPosition(actualToNativePosition(position));
    talonFXSim.setRawRotorPosition(actualToNativePosition(position));
  }

  @Override
  public void setRate(double rate) {
    talonFXSim.setRotorVelocity(actualToNativeVelocity(rate));
  }

  @Override
  public void setPositionConversion(double conversion) {
    positionConversion = conversion;
  }

  @Override
  public void setVelocityConversion(double conversion) {
    velocityConversion = conversion;
  }

  @Override
  public void setInverted(boolean inverted) {
    throw new UnsupportedOperationException("Not supported for TalonFX encoder");
  }

  @Override
  public double getPositionConversion() {
    return positionConversion;
  }

  @Override
  public double getVelocityConversion() {
    return velocityConversion;
  }
  
  @Override
  public void simulationUpdate(Optional<Double> position, Double velocity) {
    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    position.map(it -> talonFXSim.setRawRotorPosition(actualToNativePosition(it)));
    talonFXSim.setRotorVelocity(actualToNativeVelocity(velocity));
  }
}
