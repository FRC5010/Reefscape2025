// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.frc5010.common.telemetry.DisplayVoltage;

import edu.wpi.first.math.MathUtil;
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
  protected static final String EFFORT = "Output Effort";
  protected static final String TOLERANCE = "Tolerance";
  protected static final String PROFILED_MAX_VEL = "Max Velocity";
  protected static final String PROFILED_MAX_ACCEL = "Max Acceleration";

  protected DisplayString controlType;
  protected DisplayDouble feedForward;
  protected DisplayDouble position;
  protected DisplayDouble velocity;
  protected DisplayVoltage outputEffort;
  protected DisplayVoltage calculatedEffort;
  protected DisplayVoltage actualEffort;
  protected DisplayDouble kP;
  protected DisplayDouble kI;
  protected DisplayDouble kD;
  protected DisplayDouble kF;
  protected DisplayDouble iZone;
  protected DisplayDouble minOutput;
  protected DisplayDouble maxOutput;
  protected DisplayDouble reference;
  protected DisplayDouble referenceVelocity;
  protected DisplayDouble kS;
  protected DisplayDouble kV;
  protected DisplayDouble kA;
  protected DisplayDouble tolerance;
  protected DisplayDouble profiledMaxVel;
  protected DisplayDouble profiledMaxAccel;
  protected DisplayDouble outputFactor;
  protected DisplayDouble encoderFeedback;

  protected PIDController5010 controller;
  protected MotorFeedFwdConstants feedFwd;
  protected GenericEncoder encoder;

  public GenericControlledMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName);
    encoder = _motor.getMotorEncoder();
    controller = motor.getPIDController5010();
    setDisplayValuesHelper(tab);
    setMotorFeedFwd(new MotorFeedFwdConstants(kS.getValue(), kV.getValue(), kA.getValue()));
    setValues(new GenericPID(kP.getValue(), kI.getValue(), kD.getValue()));
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
    referenceVelocity = _displayValuesHelper.makeDisplayDouble("Reference Velocity");
    tolerance = _displayValuesHelper.makeConfigDouble(TOLERANCE);
    feedForward = _displayValuesHelper.makeDisplayDouble(FEEDFORWARD);
    controlType = _displayValuesHelper.makeDisplayString(CONTROL_TYPE);
    position = _displayValuesHelper.makeDisplayDouble(POSITION);
    velocity = _displayValuesHelper.makeDisplayDouble(VELOCITY);
    outputEffort = _displayValuesHelper.makeDisplayVoltage(EFFORT);
    profiledMaxVel = _displayValuesHelper.makeConfigDouble(PROFILED_MAX_VEL);
    profiledMaxAccel = _displayValuesHelper.makeConfigDouble(PROFILED_MAX_ACCEL);
    outputFactor = _displayValuesHelper.makeDisplayDouble("Output Factor");
    calculatedEffort = _displayValuesHelper.makeDisplayVoltage("Calculated Effort");
    actualEffort = _displayValuesHelper.makeDisplayVoltage("Actual Effort");
    encoderFeedback = _displayValuesHelper.makeDisplayDouble("Encoder Feedback");
  }

  @Override
  public PIDController5010 getPIDController5010() {
    return controller;
  }

  public void setPIDController5010(PIDController5010 controller) {
    this.controller = controller;
  }

  @Override
  public void setTolerance(double tolerance) {
    this.tolerance.setValue(tolerance);
    controller.setTolerance(tolerance);
  }

  @Override
  public double getTolerance() {
    return controller.getTolerance();
  }

  @Override
  public void setValues(GenericPID pidValues) {
    if (0 != pidValues.getkP())
      kP.setValue(pidValues.getkP());
    if (0 != pidValues.getkI())
      kI.setValue(pidValues.getkI());
    if (0 != pidValues.getkD())
      kD.setValue(pidValues.getkD());
    pidValues.setkP(kP.getValue());
    pidValues.setkI(kI.getValue());
    pidValues.setkD(kD.getValue());
    controller.setValues(pidValues);
  }

  @Override
  public void setP(double p) {
    kP.setValue(p);
    controller.setP(p);
  }

  @Override
  public void setI(double i) {
    kI.setValue(i);
    controller.setI(i);
  }

  @Override
  public void setD(double d) {
    kD.setValue(d);
    controller.setD(d);
  }

  @Override
  public void setF(double f) {
    kF.setValue(f);
    controller.setF(f);
  }

  @Override
  public void setIZone(double iZone) {
    this.iZone.setValue(iZone);
    controller.setIZone(iZone);
  }

  @Override
  public void setOutputRange(double min, double max) {
    minOutput.setValue(min);
    maxOutput.setValue(max);
    controller.setOutputRange(min, max);
    if (1.0 == max) {
      outputFactor.setValue(12.0);
    } else {
      outputFactor.setValue(1.0);
    }
  }

  @Override
  public void setReference(double reference) {
    this.reference.setValue(reference);
    if (GenericRobot.LogLevel.CONFIG == _displayValuesHelper.getLoggingLevel()) {
      controller.setValues(new GenericPID(kP.getValue(), kI.getValue(), kD.getValue()));
      controller.setMotorFeedFwd(new MotorFeedFwdConstants(kS.getValue(), kV.getValue(), kA.getValue()));
    }
    controller.setReference(reference);
  }

  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    this.reference.setValue(reference);
    feedForward.setValue(feedforward);
    this.controlType.setValue(controlType.name());
    if (GenericRobot.LogLevel.CONFIG == _displayValuesHelper.getLoggingLevel()) {
      // controller.setValues(new GenericPID(kP.getValue(), kI.getValue(),
      // kD.getValue()));
      // controller.setMotorFeedFwd(new MotorFeedFwdConstants(kS.getValue(),
      // kV.getValue(), kA.getValue()));
    }
    controller.setReference(reference, controlType, feedforward);
  }

  @Override
  public void setControlType(PIDControlType controlType) {
    this.controlType.setValue(controlType.name());
    controller.setControlType(controlType);
  }

  @Override
  public void setProfiledMaxVelocity(double maxVel) {
    this.profiledMaxVel.setValue(maxVel);
    controller.setProfiledMaxVelocity(maxVel);
  }

  @Override
  public void setProfiledMaxAcceleration(double maxAccel) {
    this.profiledMaxAccel.setValue(maxAccel);
    controller.setProfiledMaxAcceleration(maxAccel);
  }

  public double getProfiledMaxAcceleration() {
    return profiledMaxAccel.getValue();
  }

  public double getProfiledMaxVelocity() {
    return profiledMaxVel.getValue();
  }

  @Override
  public GenericPID getValues() {
    return controller.getValues();
  }

  /**
   * Gets the proportional value of the PID controller from the config values.
   * 
   * @return the proportional value.
   */
  @Override
  public double getP() {
    return kP.getValue();
  }

  /**
   * Gets the integral value of the PID controller from the config values.
   * 
   * @return the integral value.
   */
  @Override
  public double getI() {
    return kI.getValue();
  }

  /**
   * Gets the derivative value of the PID controller from the config values.
   * 
   * @return the derivative value.
   */
  @Override
  public double getD() {
    return kD.getValue();
  }

  @Override
  public double getF() {
    return kF.getValue();
  }

  @Override
  public double getIZone() {
    return iZone.getValue();
  }

  @Override
  public double getReference() {
    return reference.getValue();
  }

  @Override
  public PIDControlType getControlType() {
    return controller.getControlType();
  }

  @Override
  public boolean isAtTarget() {
    return controller.isAtTarget();
  }

  public abstract double getEncoderFeedback();

  public void runControllerToSetpoint() {
    double output = calculateControlEffort(encoderFeedback.getValue());
    setOutputWithFF(output, getReferenceVelocity());
  }

  public void setOutputWithFF(double power, double ffVelocity) {
    double outputFactor = this.outputFactor.getValue();
    double ff = getFeedForward(ffVelocity).in(Volts);
    double actual = MathUtil.clamp(power * outputFactor + ff, minOutput.getValue() * outputFactor,
        maxOutput.getValue() * outputFactor);
    outputEffort.setVoltage(actual, Volts);
    _motor.setVoltage(actual);
  }

  @Override
  public double calculateControlEffort(double current) {
    double controlEffort = controller.calculateControlEffort(current);
    MathUtil.clamp(controlEffort, minOutput.getValue(), maxOutput.getValue()); // clamp effort to (min, max, 0)
    calculatedEffort.setVoltage(controlEffort * outputFactor.getValue(), Volts);
    return controlEffort;
  }

  public double getVelocity() {
    return velocity.getValue();
  }

  @Override
  public double getReferenceVelocity() {
    return controller.getReferenceVelocity();
  }

  @Override
  public void resetController(double position, double velocity) {
    controller.resetController(position, velocity);
  }

  @Override
  public void setMotorFeedFwd(MotorFeedFwdConstants feedFwd) {
    if (0 != feedFwd.getkS())
      kS.setValue(feedFwd.getkS());
    if (0 != feedFwd.getkV())
      kV.setValue(feedFwd.getkV());
    if (0 != feedFwd.getkA())
      kA.setValue(feedFwd.getkA());
    feedFwd.setkS(kS.getValue());
    feedFwd.setkV(kV.getValue());
    feedFwd.setkA(kA.getValue());
    this.feedFwd = feedFwd;
    controller.setMotorFeedFwd(feedFwd);
  }

  public MotorFeedFwdConstants getMotorFeedFwd() {
    return feedFwd;
  }

  public Voltage getFeedForward(double velocity) {
    double feedforward = (null == feedFwd
        ? controller.getF()
        : controller.getReference() * (feedFwd.getkV() + feedFwd.getkA()) + feedFwd.getkS());
    feedForward.setValue(feedforward);
    return Volts.of(feedforward);
  }

  @Override
  public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
    controller.configureAbsoluteControl(offset, inverted, min, max);
  }

  public abstract Command getSysIdCommand(SubsystemBase subsystemBase);
}
