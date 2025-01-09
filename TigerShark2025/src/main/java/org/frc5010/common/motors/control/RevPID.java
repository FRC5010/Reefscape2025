// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class RevPID extends GenericPIDController {
  /** The motor for this controller */
  GenericRevBrushlessMotor motor;
  /** The PID controller */
  SparkClosedLoopController controller;
  /** The control type */
  ControlType sparkControlType = ControlType.kVoltage;
  /** The reference */
  private double reference = 0.0;
  /** The tolerance */
  private double tolerance = 0.0;
  /**
   * Configuration object for {@link SparkMax} motor.
   */
  private SparkMaxConfig cfg;
  /** The PIDF configuration */
  private GenericPID pidfConfig = new GenericPID(0, 0, 0);

  public RevPID(GenericRevBrushlessMotor motor) {
    this.motor = motor;
    controller = motor.getPIDController();
    cfg = motor.getConfig();
  }

  /**
   * Set the proportional value of the PID controller.
   *
   * @param p Proportional value.
   */
  @Override
  public void setP(double p) {
    cfg.closedLoop.p(p);
    pidfConfig.setkP(p);
    motor.updateConfig(cfg);
  }

  @Override
  public void setI(double i) {
    cfg.closedLoop.i(i);
    pidfConfig.setkI(i);
    motor.updateConfig(cfg);
  }

  @Override
  public void setD(double d) {
    cfg.closedLoop.d(d);
    pidfConfig.setkD(d);
    motor.updateConfig(cfg);  }

  @Override
  public void setF(double f) {
    pidfConfig.setkF(f);
    cfg.closedLoop.pidf(pidfConfig.getkP(), pidfConfig.getkI(), pidfConfig.getkD(), f);
    motor.updateConfig(cfg);
  }

  @Override
  public void setIZone(double iZone) {
    pidfConfig.setIZone(iZone);
    cfg.closedLoop.iZone(iZone);
    motor.updateConfig(cfg);
  }

  /**
   * Set the output range for the PID controller.
   *
   * @param min Minimum output value.
   * @param max Maximum output value.
   */
  @Override
  public void setOutputRange(double min, double max) {
    // Configure the closed loop controller's output range
    cfg.closedLoop.outputRange(min, max);
    // Update the motor configuration to apply changes
    motor.updateConfig(cfg);
  }

  @Override
  public void setReference(double reference) {
    this.reference = reference;
    controller.setReference(reference, sparkControlType);
  }

  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    setControlType(controlType);
    this.reference = reference;
    controller.setReference(feedforward, sparkControlType, ClosedLoopSlot.kSlot0, feedforward);
  }

  @Override
  public boolean isAtTarget() {
    switch (sparkControlType) {
      case kVelocity:
        return Math.abs(getReference() - motor.getMotorEncoder().getVelocity()) < tolerance;
      case kPosition:
        return Math.abs(getReference() - motor.getMotorEncoder().getPosition()) < tolerance;
      default:
        return false;
    }
  }

  @Override
  public void setTolerance(double value) {
    tolerance = value;
  }

  @Override
  public double getTolerance() {
    return tolerance;
  }

  @Override
  public void setControlType(PIDControlType controlType) {
    switch (controlType) {
      case POSITION:
        sparkControlType = ControlType.kPosition;
        break;
      case VELOCITY:
        sparkControlType = ControlType.kVelocity;
        break;
      case VOLTAGE:
        sparkControlType = ControlType.kVoltage;
        break;
      case CURRENT:
        sparkControlType = ControlType.kCurrent;
        break;
      case DUTY_CYCLE:
        sparkControlType = ControlType.kDutyCycle;
        break;
      default:
        sparkControlType = ControlType.kVoltage;
        break;
    }
  }

  @Override
  public double getP() {
    return pidfConfig.getkP();
  }

  @Override
  public double getI() {
    return pidfConfig.getkI();
  }

  @Override
  public double getD() {
    return pidfConfig.getkD();
  }

  @Override
  public double getF() {
    return pidfConfig.getkF();
  }

  @Override
  public double getIZone() {
    return pidfConfig.getIZone();
  }

  @Override
  public double getReference() {
    return reference;
  }

  @Override
  public PIDControlType getControlType() {
    switch (sparkControlType) {
      case kPosition:
        return PIDControlType.POSITION;
      case kVelocity:
        return PIDControlType.VELOCITY;
      case kVoltage:
        return PIDControlType.VOLTAGE;
      case kCurrent:
        return PIDControlType.CURRENT;
      case kDutyCycle:
        return PIDControlType.DUTY_CYCLE;
      default:
        return PIDControlType.VOLTAGE;
    }
  }

  @Override
  public void setValues(GenericPID pid) {
    setP(pid.getkP());
    setI(pid.getkI());
    setD(pid.getkD());
  }

  @Override
  public GenericPID getValues() {
    return new GenericPID(getP(), getI(), getD());
  }

  @Override
  public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
    cfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    cfg.absoluteEncoder.zeroOffset(offset);
    cfg.absoluteEncoder.inverted(inverted);
    cfg.closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(min, max);
  }
}
