// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.control.RevPID;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.RevEncoder;

import com.revrobotics.REVLibError;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** A class for a generic REV brushless motor */
public class GenericRevBrushlessMotor implements MotorController5010 {
  /**
   * {@link SparkMax} Instance.
   */
  private final SparkMax motor;
  /** The current limit */
  protected int currentLimit;
  /** The simulated instance of the motor */
  protected DCMotor motorSim;
  /** The maximum angular velocity */
  protected AngularVelocity maxRPM;
  /** The simulated instance of the motor */
  protected SparkMaxSim sparkMaxSim;

  /**
   * The maximum amount of times the swerve motor will attempt to configure a
   * motor if failures occur.
   */
  public final int maximumRetries = 5;
  /**
   * Configuration object for {@link SparkMax} motor.
   */
  private SparkMaxConfig cfg = new SparkMaxConfig();
  /**
   * Tracker for changes that need to be pushed.
   */
  private boolean cfgUpdated = false;
  /** A reference to the encoder */
  private GenericEncoder encoder = null;

  /**
   * Constructor for a generic REV brushless motor
   *
   * @param port         the port number
   * @param currentLimit the current limit
   */
  public GenericRevBrushlessMotor(int port, Current currentLimit) {
    motor = new SparkMax(port, MotorType.kBrushless);
    factoryDefaults();
    clearStickyFaults();
    setCurrentLimit(currentLimit);

    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Configure feedback of the PID controller as the
                                                                   // integrated encoder.
    cfgUpdated = true;
  }

  public GenericRevBrushlessMotor(int port, Motor config) {
    motor = new SparkMax(port, MotorType.kBrushless);
    factoryDefaults();
    clearStickyFaults();
    setCurrentLimit(config.currentLimit);
    setMotorSimulationType(config.motorSim);
    setMaxRPM(config.maxRpm);
    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Configure feedback of the PID controller as the
    // integrated encoder.
    cfgUpdated = true;
  }

  @Override
  /**
   * Sets up the same motor hardware and current limit
   *
   * @param port The port number of the motor
   */
  public MotorController5010 duplicate(int port) {
    MotorController5010 duplicate = new GenericRevBrushlessMotor(port, Amps.of(currentLimit));
    return duplicate;
  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   */
  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Units.Milliseconds.of(10).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  /**
   * Update the config for the {@link SparkMax}
   *
   * @param cfgGiven Given {@link SparkMaxConfig} which should have minimal
   *                 modifications.
   */
  public void updateConfig(SparkMaxConfig cfgGiven) {
    cfg.apply(cfgGiven);
    configureSparkMax(() -> motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    cfgUpdated = false;
  }

  /**
   * Get the current configuration of the {@link SparkMax}
   *
   * @return {@link SparkMaxConfig}
   */
  public SparkMaxConfig getConfig() {
    return cfg;
  }

  public SparkClosedLoopController getPIDController() {
    return motor.getClosedLoopController();
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public MotorController5010 setVoltageCompensation(double nominalVoltage) {
    cfg.voltageCompensation(nominalVoltage);
    cfgUpdated = true;
    return this;
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause
   * jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current
   * spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public MotorController5010 setCurrentLimit(Current currentLimit) {
    cfg.smartCurrentLimit((int) currentLimit.in(Amps));
    cfgUpdated = true;
    return this;
  }

  /**
   * Sets the slew rate for the motor controller.
   *
   * @param rate the slew rate to be set
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setSlewRate(double rate) {
    cfg.closedLoopRampRate(rate)
        .openLoopRampRate(rate);
    cfgUpdated = true;

    return this;
  }

  /**
   * Sets the motor as a follower of another motor.
   *
   * @param motor the motor to follow
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor) {
    cfg.follow((SparkBase) motor.getMotor());
    cfgUpdated = true;
    return this;
  }

  /**
   * Sets the motor as a follower of another motor, with the option to invert the
   * output.
   *
   * @param motor    the motor to follow
   * @param inverted whether the output should be inverted
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
    cfg.follow((SparkBase) motor.getMotor(), inverted);
    cfgUpdated = true;
    return this;
  }

  /**
   * Inverts the motor.
   *
   * @param inverted a boolean indicating whether the motor should be inverted
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 invert(boolean inverted) {
    configureSparkMax(() -> {
      cfg.inverted(inverted);
      return motor.getLastError();
    });
    return this;
  }

  /**
   * Retrieves the generic encoder for the motor.
   *
   * @return a new instance of GenericEncoder, which wraps the encoder returned by
   *         the superclass's
   *         getEncoder() method
   */
  @Override
  public GenericEncoder getMotorEncoder() {
    if (null == encoder) {
      encoder = new RevEncoder(motor.getEncoder());
    }
    return encoder;
  }

  /**
   * Retrieves the generic encoder for the motor with the specified sensor type
   * and counts per
   * revolution.
   *
   * @param countsPerRev the number of counts per revolution of the motor
   * @return a new instance of GenericEncoder, which wraps the encoder returned by
   *         the superclass's
   *         getEncoder() method
   */
  @Override
  public GenericEncoder getMotorEncoder(int countsPerRev) {
    getMotorEncoder();
    encoder.setPositionConversion(countsPerRev);
    encoder.setVelocityConversion(countsPerRev / 60.0);
    return encoder;
  }

  /**
   * Returns a new instance of PIDController5010, which is a wrapper around the
   * RevPID class.
   *
   * @return a new instance of PIDController5010
   */
  @Override
  public PIDController5010 getPIDController5010() {
    return new RevPID(this);
  }

  /**
   * Returns the MotorController instance that this object represents.
   *
   * @return the MotorController instance that this object represents
   */
  @Override
  public Object getMotor() {
    return motor;
  }

  /**
   * Returns a SysIdRoutine instance that represents the default system
   * identification routine for
   * this object.
   *
   * @param subsystemBase the subsystem that this routine is associated with
   * @return a SysIdRoutine instance that represents the default system
   *         identification routine
   */
  @Override
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    return SystemIdentification.rpmSysIdRoutine(
        this, getMotorEncoder(), "Motor " + motor.getDeviceId(), subsystemBase);
  }

  /**
   * Overrides the factoryDefault method to restore the default settings of the
   * superclass. This
   * method is called to restore the default settings of the motor controller.
   */
  @Override
  public void factoryDefaults() {
  }

  /**
   * Returns the motor simulation type as a {@link DCMotor} instance.
   *
   * @return the simulated instance of the motor for use in simulations
   */
  @Override
  public DCMotor getMotorSimulationType() {
    return motorSim;
  }

  /**
   * Returns the maximum angular velocity of the motor in rotations per minute.
   *
   * @return the maximum angular velocity of the motor in rotations per minute
   */
  @Override
  public AngularVelocity getMaxRPM() {
    return maxRPM;
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults() {
    configureSparkMax(motor::clearFaults);
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public MotorController5010 setMotorBrake(boolean isBrakeMode) {
    cfg.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    cfgUpdated = true;
    return this;
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }
    motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    cfgUpdated = false;
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the applied dutycycle output.
   *
   * @return Applied dutycycle output to the motor.
   */
  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  /**
   * Gets the motor output as a percentage
   * 
   * @return The motor output
   */
  @Override
  public double get() {
    return motor.get();
  }

  /**
   * Set the motor to be inverted.
   *
   * @param isInverted State of inversion.
   */
  @Override
  public void setInverted(boolean isInverted) {
    invert(isInverted);
  }

  /**
   * Returns the inversion status of the motor.
   *
   * @return true if the motor is inverted, false otherwise
   */
  @Override
  public boolean getInverted() {
    throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
  }

  /**
   * Disables the motor by calling the disable method on the underlying motor
   * object.
   */
  @Override
  public void disable() {
    motor.disable();
  }

  /**
   * Stops the motor by calling the stopMotor method on the underlying motor
   * object.
   *
   * @see MotorController5010#stopMotor()
   */
  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Get the current output of the motor controller.
   * 
   * @return Current output.
   */
  @Override
  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Sets the simulated instance of the motor for use in simulations.
   *
   * @param motorSimulationType The simulated instance of the motor.
   */
  @Override
  public void setMotorSimulationType(DCMotor motorSimulationType) {
    motorSim = motorSimulationType;
    sparkMaxSim = new SparkMaxSim(motor, motorSim);
  }

  /**
   * Sets the maximum angular velocity of the motor in rotations per minute.
   *
   * @param rpm The maximum angular velocity of the motor in rotations per minute.
   */
  @Override
  public void setMaxRPM(AngularVelocity rpm) {
    maxRPM = rpm;
  }

  @Override
  public void simulationUpdate(Optional<Angle> position, AngularVelocity velocity) {
    sparkMaxSim.iterate(velocity.in(RPM), RoboRioSim.getVInVoltage(), 0.02);
  }

}
