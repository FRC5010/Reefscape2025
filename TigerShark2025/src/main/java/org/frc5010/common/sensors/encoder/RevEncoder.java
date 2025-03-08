// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.encoder;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class RevEncoder implements GenericEncoder {
  protected RelativeEncoder encoder;
    /** The simulated instance of the motor */
  protected SparkMaxSim sparkMaxSim;
  protected EncoderConfig config;
  protected double positionConversion = 1.0;
  protected double velocityConversion = 1.0;

  public RevEncoder(RelativeEncoder encoder) {
    this.encoder = encoder;
    config = new EncoderConfig();
  }

  public void setSimulation(SparkMaxSim sparkMaxSim) {
    this.sparkMaxSim = sparkMaxSim;
  }

  @Override
  public double getPosition() {
    if (RobotBase.isReal()) {
      return encoder.getPosition();
    } else {
      return sparkMaxSim.getPosition();
    }
  }

  @Override
  public double getVelocity() {
    if (RobotBase.isReal()) {
      return encoder.getVelocity();
    } else {
      return sparkMaxSim.getVelocity();
    }
  }

  @Override
  public void reset() {
    setPosition(0);

  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
    sparkMaxSim.getRelativeEncoderSim().setPosition(position);
  }

  @Override
  public void setRate(double rate) {
    sparkMaxSim.getRelativeEncoderSim().setVelocity(rate);
  }

  @Override
  public void setPositionConversion(double conversion) {
    positionConversion = conversion;
    config.positionConversionFactor(conversion);
    //sparkMaxSim.getRelativeEncoderSim().setPositionConversionFactor(conversion);
    
  }

  @Override
  public void setVelocityConversion(double conversion) {
    velocityConversion = conversion;
    config.velocityConversionFactor(conversion);
    //sparkMaxSim.getRelativeEncoderSim().setVelocityConversionFactor(conversion);
    
  }

  @Override
  public void setInverted(boolean inverted) {
    sparkMaxSim.getRelativeEncoderSim().setInverted(inverted);
    config.inverted(inverted);
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
    sparkMaxSim.iterate(velocity, RoboRioSim.getVInVoltage(), 0.02);
  }
}
