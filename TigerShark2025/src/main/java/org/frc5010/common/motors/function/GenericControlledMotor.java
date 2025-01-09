// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.frc5010.common.telemetry.DisplayVoltage;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class GenericControlledMotor extends GenericFunctionalMotor
    implements PIDController5010 {
  protected static final String K_P = "kP";
  protected static final String K_I = "kI";
  protected static final String K_D = "kD";
  protected static final String K_F = "kF";
  protected static final String K_S = "kS";
  protected static final String K_V = "kV";
  protected static final String K_A = "kA";
  protected static final String I_ZONE = "IZone";
  protected static final String MIN_OUTPUT = "MinOutput";
  protected static final String MAX_OUTPUT = "MaxOutput";
  protected static final String REFERENCE = "Reference";
  protected static final String CONTROL_TYPE = "ControlType";
  protected static final String FEEDFORWARD = "FeedForward";
  protected static final String CONFIG_MODE = "ConfigMode";
  protected static final String POSITION = "Position";
  protected static final String VELOCITY = "Velocity";
  protected static final String EFFORT = "Effort";
  protected static final String TOLERANCE = "Tolerance";

  protected DisplayString controlType;
  protected DisplayDouble feedForward;
  protected DisplayDouble position;
  protected DisplayDouble velocity;
  protected DisplayVoltage effort;
  protected DisplayDouble kP;
  protected DisplayDouble kI;
  protected DisplayDouble kD;
  protected DisplayDouble kF;
  protected DisplayDouble iZone;
  protected DisplayDouble minOutput;
  protected DisplayDouble maxOutput;
  protected DisplayDouble reference;
  protected DisplayDouble kS;
  protected DisplayDouble kV;
  protected DisplayDouble kA;
  protected DisplayDouble tolerance;

  protected PIDController5010 pid;
  protected MotorFeedFwdConstants feedFwd;
  protected GenericEncoder encoder;

  public GenericControlledMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName);
    encoder = _motor.getMotorEncoder();
    pid = motor.getPIDController5010();
    setDisplayValuesHelper(tab);
  }

  /**
   * Initializes all the display values for the motor, including PID values, feed
   * forward, and control type. This function is called by the superclass's
   * constructor.
   */
  @Override
  protected void initiateDisplayValues() {
    kP = _displayValuesHelper.makeConfigDouble(K_P);
    kI = _displayValuesHelper.makeConfigDouble(K_I);
    kD = _displayValuesHelper.makeConfigDouble(K_D);
    kF = _displayValuesHelper.makeConfigDouble(K_F);
    kS = _displayValuesHelper.makeConfigDouble(K_S);
    kV = _displayValuesHelper.makeConfigDouble(K_V);
    kA = _displayValuesHelper.makeConfigDouble(K_A);
    iZone = _displayValuesHelper.makeConfigDouble(I_ZONE);
    minOutput = _displayValuesHelper.makeConfigDouble(MIN_OUTPUT);
    maxOutput = _displayValuesHelper.makeConfigDouble(MAX_OUTPUT);
    reference = _displayValuesHelper.makeDisplayDouble(REFERENCE);
    tolerance = _displayValuesHelper.makeConfigDouble(TOLERANCE);
    feedForward = _displayValuesHelper.makeDisplayDouble(FEEDFORWARD);
    controlType = _displayValuesHelper.makeDisplayString(CONTROL_TYPE);
    position = _displayValuesHelper.makeDisplayDouble(POSITION);
    velocity = _displayValuesHelper.makeDisplayDouble(VELOCITY);
    effort = _displayValuesHelper.makeDisplayVoltage(EFFORT);
  }
  
  @Override
  public PIDController5010 getPIDController5010() {
    return pid;
  }

  @Override
  public void setTolerance(double tolerance) {
    this.tolerance.setValue(tolerance);
    pid.setTolerance(tolerance);
  }

  @Override
  public double getTolerance() {
    return pid.getTolerance();
  }

  @Override
  public void setValues(GenericPID pidValues) {
    kP.setValue(pidValues.getkP());
    kI.setValue(pidValues.getkI());
    kD.setValue(pidValues.getkD());
    pid.setValues(pidValues);
  }

  @Override
  public void setP(double p) {
    kP.setValue(p);
    pid.setP(p);
  }

  @Override
  public void setI(double i) {
    kI.setValue(i);
    pid.setI(i);
  }

  @Override
  public void setD(double d) {
    kD.setValue(d);
    pid.setD(d);
  }

  @Override
  public void setF(double f) {
    kF.setValue(f);
    pid.setF(f);
  }

  @Override
  public void setIZone(double iZone) {
    this.iZone.setValue(iZone);
    pid.setIZone(iZone);
  }

  @Override
  public void setOutputRange(double min, double max) {
    minOutput.setValue(min);
    maxOutput.setValue(max);
    pid.setOutputRange(min, max);
  }

  @Override
  public void setReference(double reference) {
    this.reference.setValue(reference);
    pid.setReference(reference);
  }

  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    this.reference.setValue(reference);
    feedForward.setValue(feedforward);
    this.controlType.setValue(controlType.name());
    pid.setReference(reference, controlType, feedforward);
  }

  @Override
  public void setControlType(PIDControlType controlType) {
    this.controlType.setValue(controlType.name());
    pid.setControlType(controlType);
  }

  @Override
  public GenericPID getValues() {
    return pid.getValues();
  }

  @Override
  public double getP() {
    return pid.getP();
  }

  @Override
  public double getI() {
    return pid.getI();
  }

  @Override
  public double getD() {
    return pid.getD();
  }

  @Override
  public double getF() {
    return pid.getF();
  }

  @Override
  public double getIZone() {
    return pid.getIZone();
  }

  @Override
  public double getReference() {
    return pid.getReference();
  }

  @Override
  public PIDControlType getControlType() {
    return pid.getControlType();
  }

  @Override
  public boolean isAtTarget() {
    return pid.isAtTarget();
  }

  @Override
  public double calculateControlEffort(double current) {
    double controlEffort = pid.calculateControlEffort(current) + getFeedForward().in(Volts);
    if (controlEffort > maxOutput.getValue()) {
      controlEffort = maxOutput.getValue();
    } else if (controlEffort < minOutput.getValue()) {
      controlEffort = minOutput.getValue();
    }
    effort.setVoltage(controlEffort, Volts);
    return controlEffort;
  }

  public void setMotorFeedFwd(MotorFeedFwdConstants feedFwd) {
    this.feedFwd = feedFwd;
    kS.setValue(feedFwd.getkS());
    kV.setValue(feedFwd.getkV());
    kA.setValue(feedFwd.getkA());
  }

  public MotorFeedFwdConstants getMotorFeedFwd() {
    return feedFwd;
  }

  public Voltage getFeedForward() {
    double feedforward =
        (null == feedFwd
            ? 0.0
            : pid.getReference() * (feedFwd.getkV() + feedFwd.getkA()) + feedFwd.getkS());
    feedForward.setValue(feedforward);
    return Volts.of(feedforward);
  }

  @Override
  public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
    pid.configureAbsoluteControl(offset, inverted, min, max);
  }

  public abstract Command getSysIdCommand(SubsystemBase subsystemBase);
}
