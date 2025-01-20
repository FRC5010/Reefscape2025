// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.control.TalonFXPID;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.TalonFXEncoder;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** A class for a generic TalonFX motor */
public class GenericTalonFXMotor implements MotorController5010 {
  /**
   * Wait time for status frames to show up.
   */
  public static double STATUS_TIMEOUT_SECONDS = 0.02;
  /**
   * Factory default already occurred.
   */
  private final boolean factoryDefaultOccurred = false;
  /**
   * TalonFX motor controller.
   */
  private final TalonFX motor;
  /**
   * Current TalonFX configuration.
   */
  private TalonFXConfiguration configuration = new TalonFXConfiguration();
  /**
   * Current TalonFX Configurator.
   */
  private TalonFXConfigurator cfg;
  /** Current motor current limit */
  protected int motorCurrentLimit;
  /** Current controller current limit */
  protected int controllerCurrentLimit;
  /** Enable FOC */
  protected boolean enableFOC = true;

  /** DCMotor simulation */
  protected DCMotor motorSim;

  /** TalonFX simulation */
  protected TalonFXSimState talonFXSim;

  protected AngularVelocity maxRPM;

  /**
   * Constructor for TalonFX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public GenericTalonFXMotor(TalonFX motor) {
    this.motor = motor;
    this.cfg = motor.getConfigurator();

    factoryDefaults();
    clearStickyFaults();
  }

  /**
   * Construct the TalonFX swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param canbus       CANBus on which the TalonFX is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public GenericTalonFXMotor(int id, String canbus) {
    this(new TalonFX(id, canbus));
  }

  /**
   * Construct the TalonFX swerve motor given the ID and CANBus.
   *
   * @param id           ID of the TalonFX on the CANBus.
   * @param canbus       CANBus on which the TalonFX is on.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public GenericTalonFXMotor(int id) {
    this(new TalonFX(id));
  }

  /**
   * Creates a duplicate of the current motor controller with the specified port.
   *
   * @param port The port number for the new motor controller.
   * @return A new instance of MotorController5010 with the same configuration.
   */
  @Override
  public MotorController5010 duplicate(int port) {
    MotorController5010 duplicate = new GenericTalonFXMotor(port);
    return duplicate;
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults() {
    if (!factoryDefaultOccurred) {
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configuration.ClosedLoopGeneral.ContinuousWrap = true;
      cfg.apply(configuration);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause
   * jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current
   * spikes.
   *
   * @param limit Current limit in AMPS at free speed.
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setCurrentLimit(Current limit) {
    motorCurrentLimit = (int)limit.in(Amps);

    cfg.refresh(configuration.CurrentLimits);
    cfg.apply(
        configuration.CurrentLimits.withSupplyCurrentLimit(motorCurrentLimit)
            .withSupplyCurrentLimitEnable(true));

    return this;
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rate Time in seconds to go from 0 to full throttle.
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setSlewRate(double rate) {
    cfg.refresh(configuration.ClosedLoopRamps);
    cfg.apply(configuration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(rate));
    return this;
  }

  /**
   * Set the motor to follow another motor controller. This is useful for
   * master/slave
   * configurations or for having a motor follow another motor without having to
   * duplicate
   * code.
   *
   * @param motor The motor controller to follow.
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor) {
    this.motor.setControl(new Follower(((TalonFX) motor).getDeviceID(), false));
    return this;
  }

  /**
   * Set the motor to follow another motor controller with the option to invert
   * the follower.
   *
   * @param motor    The motor controller to follow.
   * @param inverted Whether the follower should be inverted or not.
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
    this.motor.setControl(new Follower(((TalonFX) motor).getDeviceID(), inverted));
    return this;
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted) {
    // Timer.delay(1);
    cfg.refresh(configuration.MotorOutput);
    configuration.MotorOutput.withInverted(
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
    cfg.apply(configuration.MotorOutput);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted Whether the motor should be inverted.
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 invert(boolean inverted) {
    setInverted(inverted);
    return this;
  }

  /**
   * Retrieves the motor encoder for the TalonFX motor.
   *
   * @return an instance of GenericEncoder, specifically a TalonFXEncoder
   *         associated with this motor.
   */
  @Override
  public GenericEncoder getMotorEncoder() {
    return new TalonFXEncoder(this);
  }

  /**
   * Retrieves the motor encoder for the TalonFX motor, with the specified counts
   * per
   * revolution.
   *
   * @param countsPerRev The number of counts per revolution of the motor
   * @return an instance of GenericEncoder, specifically a TalonFXEncoder
   *         associated with
   *         this motor.
   */
  @Override
  public GenericEncoder getMotorEncoder(int countsPerRev) {
    GenericEncoder encoder = new TalonFXEncoder(this);
    encoder.setPositionConversion(countsPerRev);
    encoder.setVelocityConversion(countsPerRev);
    return encoder;
  }

  /**
   * Retrieves the PID controller specific to the TalonFX motor.
   *
   * @return an instance of PIDController5010, specifically a TalonFXPID
   *         associated with this motor.
   */
  @Override
  public PIDController5010 getPIDController5010() {
    return new TalonFXPID(this);
  }

  /**
   * Retrieves the motor controller itself.
   *
   * @return The motor controller (this object)
   */
  @Override
  public Object getMotor() {
    return motor;
  }

  /**
   * Returns the default system identification routine for the motor.
   *
   * This method creates a SysIdRoutine configured to prevent motor brownout by
   * reducing the dynamic voltage to 4 volts. It uses the Phoenix SignalLogger
   * class to log the state of the system. The mechanism implementation sets
   * the motor control using the provided voltage output and is associated
   * with the given subsystem.
   *
   * @param subsystemBase The subsystem that this routine is associated with
   * @return A SysIdRoutine instance configured for system identification
   */
  @Override
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    VoltageOut sysidControl = new VoltageOut(0);
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Default ramp rate is acceptable
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
            null, // Default timeout is acceptable
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> motor.setControl(sysidControl.withOutput(volts.in(Volts))),
            null,
            subsystemBase));
  }

  /**
   * Checks if Field-Oriented Control (FOC) is enabled for the motor.
   *
   * @return true if FOC is enabled, false otherwise
   */
  public boolean isFOCEnabled() {
    return enableFOC;
  }

  /**
   * Enable or disable Field-Oriented Control (FOC) for the motor. FOC allows
   * the motor to operate more efficiently by adjusting the motor's magnetic
   * field, but it also requires a position sensor and may not work with all
   * motor types. FOC is disabled by default.
   *
   * @param enableFOC true to enable FOC, false to disable
   */
  public void enableFOC(boolean enableFOC) {
    this.enableFOC = enableFOC;
  }

  /**
   * Returns the simulation type of the motor as a {@link DCMotor}.
   *
   * @return A {@link DCMotor} that represents the simulation type of the motor.
   * @throws UnsupportedOperationException if the method is not implemented.
   */
  @Override
  public DCMotor getMotorSimulationType() {
    throw new UnsupportedOperationException("Unimplemented method 'getMotorSimulationType'");
  }

  /**
   * Returns the maximum revolutions per minute (RPM) the motor can achieve.
   *
   * @return The maximum RPM of the motor.
   * @throws UnsupportedOperationException if the method is not implemented.
   */
  @Override
  public AngularVelocity getMaxRPM() {
    throw new UnsupportedOperationException("Unimplemented method 'getMaxRPM'");
  }

  /**
   * Sets the voltage compensation of the motor to the given nominal voltage.
   *
   * <p>
   * This method does not do anything for TalonFX motors, but is included to
   * provide a consistent interface with other motor types.
   *
   * @param nominalVoltage The nominal voltage to set for voltage compensation.
   * @return This motor controller.
   */
  @Override
  public MotorController5010 setVoltageCompensation(double nominalVoltage) {
    return this;
  }

  /**
   * Sets the idle mode of the motor to either brake or coast.
   *
   * @param isBrakeMode If true, sets the motor to brake mode; otherwise, sets it
   *                    to coast mode.
   * @return This motor controller instance.
   */
  @Override
  public MotorController5010 setMotorBrake(boolean isBrakeMode) {
    motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    return this;
  }

  /**
   * Saves the motor controller's configuration to flash memory. This must be
   * called after setting the motor controller's configuration in order to
   * persist the changes after the robot is restarted.
   */
  @Override
  public void burnFlash() {
    cfg.apply(configuration);
  }

  /**
   * Retrieves the voltage output of the motor controller.
   *
   * This method waits for the voltage update within a specified timeout
   * and returns the motor's voltage in volts.
   *
   * @return The voltage output of the motor in volts.
   */
  @Override
  public double getVoltage() {
    return motor.getMotorVoltage().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue().in(Volts);
  }

  /**
   * Gets the applied output of the motor as a double in the range of -1.0 to 1.0.
   *
   * <p>
   * This method waits for the applied output update within a specified timeout
   * and returns the motor's applied output as a double between -1 and 1.
   *
   * @return The motor's applied output as a double between -1 and 1.
   */
  @Override
  public double getAppliedOutput() {
    return motor.getDutyCycle().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValue();
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
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the current output of the motor controller.
   *
   * @return The motor output as a double between -1 and 1.
   */
  @Override
  public double get() {
    return motor.get();
  }

  /**
   * Returns the inversion status of the motor.
   *
   * <p>
   * This method checks the current motor output configuration and determines
   * if the motor is set to be inverted.
   *
   * @return true if the motor is inverted (i.e., configured for counterclockwise
   *         positive rotation), false if the motor is not inverted (i.e.,
   *         configured for
   *         clockwise positive rotation).
   */
  @Override
  public boolean getInverted() {
    cfg.refresh(configuration.MotorOutput);
    return (configuration.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive ? true : false);
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
   * Stop the motor by setting the output to 0.
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
    return motor.getTorqueCurrent().waitForUpdate(STATUS_TIMEOUT_SECONDS).getValueAsDouble();
  }

  /**
   * Sets the simulated instance of the motor for use in simulations.
   *
   * @param motorSimulationType The simulated instance of the motor.
   */
  @Override
  public void setMotorSimulationType(DCMotor motorSimulationType) {
    motorSim = motorSimulationType;
    talonFXSim = motor.getSimState();
  }

  /**
   * Update the motor simulation model with the current state of the motor.
   *
   * @param position The current angle of the motor in radians.
   * @param velocity The current angular velocity of the motor in radians per
   *                 second.
   */
  @Override
  public void simulationUpdate(Optional<Angle> position, AngularVelocity velocity) {
    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    position.map(it -> talonFXSim.setRawRotorPosition(it.in(Rotations)));
    talonFXSim.setRotorVelocity(velocity.in(RPM));
  }

  /**
   * Sets the maximum angular velocity of the motor in rotations per minute.
   *
   * @param rpm The maximum angular velocity to set, represented as an
   *            AngularVelocity unit.
   */
  @Override
  public void setMaxRPM(AngularVelocity rpm) {
    maxRPM = rpm;
  }
}
