// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;
import org.frc5010.common.sensors.encoder.TalonFXEncoder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class TalonFXController extends GenericPIDController {
  protected TalonFX internalMotor;
  protected GenericTalonFXMotor motor;
  protected ControlRequest request;
  protected PIDControlType controlType = PIDControlType.DUTY_CYCLE;
  /**
   * Current TalonFX configuration.
   */
  private TalonFXConfiguration configuration = new TalonFXConfiguration();
  /**
   * Current TalonFX Configurator.
   */
  private TalonFXConfigurator cfg;
  protected double reference = 0.0;
  protected double tolerance;

  public TalonFXController(GenericTalonFXMotor motor) {
    this.motor = motor;
    internalMotor = (TalonFX) motor.getMotor();
    cfg = internalMotor.getConfigurator();
    refreshTalonConfigs();
    setControlType(controlType);
  }

  /**
   * Refreshes the TalonFX configuration to ensure we have the latest PID
   * configuration.
   */
  private void refreshTalonConfigs() {
    cfg.refresh(configuration);
  }

  /**
   * Sends a control request to the TalonFX motor based on the set control type
   * and
   * reference point.
   *
   * @param reference   The reference point to be set. Units depend on the control
   *                    type.
   * @param feedforward The feedforward value to be used.
   */
  private void sendControlRequest(double reference, double feedforward) {
    switch (controlType) {
      case NONE: return;
      case POSITION:
        ((PositionVoltage) request).withPosition(reference)
            .withFeedForward(feedforward);
        break;
      case VELOCITY:
        ((VelocityVoltage) request).withVelocity(reference)
            .withFeedForward(feedforward)
            .withEnableFOC(motor.isFOCEnabled());
        break;
      case DUTY_CYCLE:
        ((DutyCycleOut) request).withOutput(reference);
        break;
      case VOLTAGE:
        ((VoltageOut) request).withOutput(Volts.of(reference));
        break;
      case PROFILED_POSITION:
        ((MotionMagicVoltage) request).withPosition(reference)
            .withFeedForward(feedforward);
        break;
      case PROFILED_VELOCITY:
        ((MotionMagicVelocityVoltage) request).withVelocity(reference)
            .withFeedForward(feedforward);
        break;
      default:
        throw new IllegalArgumentException("Unsupported TalonFX control type");
    }
    internalMotor.setControl(request);
    
  }

  /**
   * Checks if the motor is at the target setpoint. If the motor is being
   * controlled in
   * velocity control mode, the absolute difference between the current velocity
   * and the
   * target velocity is checked against the tolerance. If the motor is being
   * controlled in
   * position control mode, the absolute difference between the current position
   * and the
   * target position is checked against the tolerance. If the motor is being
   * controlled in
   * duty cycle control mode, the function returns false.
   *
   * @return true if the motor is at the target setpoint, false otherwise.
   */
  @Override
  public boolean isAtTarget() {
      switch (controlType) {
        case VELOCITY:
          double velocity = internalMotor.getVelocity().getValueAsDouble();
          return Math.abs(getReference() - velocity) < tolerance;
        case POSITION:
          double position = internalMotor.getPosition().getValueAsDouble();
          return Math.abs(getReference() - position) < tolerance;
        default:
          return false;
      }
  }

  /**
   * Sets the tolerance value for the PID controller.
   *
   * @param value The tolerance value to be set.
   */
  @Override
  public void setTolerance(double value) {
    tolerance = value;
  }

  /**
   * Retrieves the tolerance value for the PID controller.
   *
   * @return The tolerance value as a double.
   */
  @Override
  public double getTolerance() {
    return tolerance;
  }

  /**
   * Sets the PID configuration values for the TalonFX motor.
   *
   * <p>
   * This function will take the PID values and apply them to the TalonFX motor.
   * The
   * kP, kI, kD, and kS constants are set using the {@link TalonFXConfigurator}
   * class.
   *
   * @param pid the PID values to set.
   */
  @Override
  public void setValues(GenericPID pid) {
    cfg.refresh(configuration.Slot0);
    cfg.apply(
        configuration.Slot0.withKP(pid.getkP())
            .withKI(pid.getkI())
            .withKD(pid.getkD()));
  }

  /**
   * Not implemented.
   * 
   * @param f the feedforward value.
   */
  @Override
  public void setF(double f) {
  }

  /**
   * Sets the integral zone for the PID controller.
   *
   * @param iZone the integral zone value to be set.
   */
  @Override
  public void setIZone(double iZone) {
  }

  @Override
  public void setReference(double reference) {
    setReference(reference, controlType, 0.0);
  }

  /**
   * Sets the reference point of the PID controller.
   *
   * @param reference   The reference point to be set. Units depend on the control
   *                    type.
   * @param controlType The type of control to be used.
   * @param feedforward The feedforward value to be used.
   */
  @Override
  public void setReference(double reference, PIDControlType controlType, double feedforward) {
    this.reference = ((TalonFXEncoder) motor.getMotorEncoder()).distanceToRotations(reference);
    setControlType(controlType);
    sendControlRequest(reference, feedforward);
  }

  /**
   * Sets the control type of the PID controller.
   *
   * @param controlType the PID control type to be used.
   */
  @Override
  public void setControlType(PIDControlType controlType) {
    if (null == this.request || this.controlType != controlType) {
      switch (controlType) {
        case NONE:
          break;
        case POSITION:
          request = new PositionVoltage(reference)
              .withEnableFOC(motor.isFOCEnabled());
          break;
        case VELOCITY:
          request = new VelocityVoltage(reference)
              .withEnableFOC(motor.isFOCEnabled());
          break;
        case DUTY_CYCLE:
          request = new DutyCycleOut(reference).withEnableFOC(motor.isFOCEnabled());
          break;
        case VOLTAGE:
          request = new VoltageOut(reference).withEnableFOC(motor.isFOCEnabled());
          break;
        case PROFILED_POSITION:
          request = new MotionMagicVoltage(reference).withEnableFOC(motor.isFOCEnabled());
          break;
        case PROFILED_VELOCITY:
          request = new MotionMagicVelocityVoltage(reference).withEnableFOC(motor.isFOCEnabled());
          break;
        default:
          throw new IllegalArgumentException("Unsupported TalonFX control type " + controlType);
      }
    }
    this.controlType = controlType;
  }

  /**
   * Retrieves the current PID configuration values.
   *
   * @return A GenericPID object containing the proportional (kP), integral (kI),
   *         derivative (kD), and feedforward (kS) constants.
   */
  @Override
  public GenericPID getValues() {
    GenericPID pidConfig = new GenericPID(configuration.Slot0.kP, configuration.Slot0.kI, configuration.Slot0.kD);
    return pidConfig;
  }

  /**
   * Retrieves the proportional value of the PID controller.
   *
   * @return The proportional constant (kP) of the PID controller.
   */
  @Override
  public double getP() {
    refreshTalonConfigs();
    return configuration.Slot0.kP;
  }

  /**
   * Retrieves the integral value of the PID controller.
   *
   * @return The integral constant (kI) of the PID controller.
   */
  @Override
  public double getI() {
    refreshTalonConfigs();
    return configuration.Slot0.kI;
  }

  /**
   * Retrieves the derivative value of the PID controller.
   *
   * @return The derivative constant (kD) of the PID controller.
   */
  @Override
  public double getD() {
    refreshTalonConfigs();
    return configuration.Slot0.kD;
  }

  /**
   * Retrieves the feedforward value of the PID controller.
   *
   * @return The feedforward constant (kS) of the PID controller.
   */
  @Override
  public double getF() {
    return 0;
  }

  /**
   * Retrieves the integral zone value of the PID controller.
   *
   * @return The integral zone value (iZone) of the PID controller.
   */
  @Override
  public double getIZone() {
    return 0;
  }

  /**
   * Retrieves the current reference point of the PID controller.
   *
   * @return The reference point as a double.
   */
  @Override
  public double getReference() {
    return reference;
  }

  /**
   * Retrieves the current control type of the PID controller.
   *
   * @return The control type of the PID controller.
   */
  @Override
  public PIDControlType getControlType() {
    return controlType;
  }

  /**
   * Sets the proportional value of the PID controller.
   *
   * @param p Proportional value.
   */
  @Override
  public void setP(double p) {
    refreshTalonConfigs();
    configuration.Slot0.kP = p;
    cfg.apply(configuration.Slot0);
  }

  /**
   * Sets the integral value of the PID controller.
   *
   * @param i Integral value.
   */
  @Override
  public void setI(double i) {
    refreshTalonConfigs();
    configuration.Slot0.kI = i;
    cfg.apply(configuration.Slot0);
  }

  /**
   * Sets the derivative value of the PID controller.
   *
   * @param d Derivative value.
   */
  @Override
  public void setD(double d) {
    refreshTalonConfigs();
    configuration.Slot0.kD = d;
    cfg.apply(configuration.Slot0);
  }

  /**
   * Sets the minimum and maximum output range for the PID controller.
   *
   * @param min The minimum output value.
   * @param max The maximum output value.
   * @throws UnsupportedOperationException if the method is not implemented.
   */
  @Override
  public void setOutputRange(double min, double max) { // TODO: Implement
    //throw new UnsupportedOperationException("Not implemented for TalonFX");
  }

  /**
   * Configure the PID controller for absolute control. This is the same as
   * absolute encoder control, but for the TalonFX.
   *
   * @param offset   The offset of the absolute position sensor.
   * @param inverted Whether the sensor is inverted or not.
   * @param min      The minimum position value.
   * @param max      The maximum position value.
   */
  @Override
  public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
    setControlType(PIDControlType.POSITION);
    cfg.refresh(configuration.ClosedLoopGeneral);
    configuration.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.apply(configuration.ClosedLoopGeneral);
  }

  @Override
  public void setProfiledMaxVelocity(double maxVelocity) {
    cfg.refresh(configuration.MotionMagic);
    configuration.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    cfg.apply(configuration.MotionMagic);
  }

  @Override
  public void setProfiledMaxAcceleration(double maxAcceleration) {
    cfg.refresh(configuration.MotionMagic);
    configuration.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    //configuration.MotionMagic.MotionMagicJerk = ((TalonFXEncoder) motor.getMotorEncoder()).velocityToRotationsPerMin(maxAcceleration) / 3600.0;
    cfg.apply(configuration.MotionMagic);
  }

  @Override
  public void setMotorFeedFwd(MotorFeedFwdConstants motorConstants) {
    cfg.refresh(configuration.Slot0);
    configuration.Slot0.kS = motorConstants.getkS();
    configuration.Slot0.kV = motorConstants.getkV();
    configuration.Slot0.kA = motorConstants.getkA();
    cfg.apply(configuration.Slot0);
  }

  public void applyConfig() {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = cfg.apply(configuration);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }
}
