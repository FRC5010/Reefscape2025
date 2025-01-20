// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.frc5010.common.arch.WpiHelperInterface;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** A class that wraps a motor controller with functionality */
public class GenericFunctionalMotor implements MotorController5010, WpiHelperInterface {
  /** The motor */
  protected MotorController5010 _motor;

  protected Mechanism2d _visualizer;
  protected Pose3d _robotToMotor;
  protected String _visualName;
  protected DisplayValuesHelper _displayValuesHelper;

  /**
   * Constructor for a motor
   *
   * @param motor The motor
   */
  public GenericFunctionalMotor(MotorController5010 motor, String visualName) {
    this._motor = motor;
    _visualName = visualName;
  }

  /**
   * Constructor for a motor
   *
   * @param motor    The motor
   * @param slewRate The slew rate
   */
  public GenericFunctionalMotor(MotorController5010 motor, double slewRate) {
    this._motor = motor;
    _motor.setSlewRate(slewRate);
  }

  /**
   * Duplicates the motor controller and returns a new instance with the specified
   * port.
   *
   * @param port the port number for the new motor controller
   * @return a new instance of MotorController5010 with the specified port
   */
  @Override
  public MotorController5010 duplicate(int port) {
    return _motor.duplicate(port);
  }

  /**
   * Sets the current limit for the motor controller.
   *
   * @param limit the current limit to be set
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setCurrentLimit(Current limit) {
    _motor.setCurrentLimit(limit);
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
    _motor.setSlewRate(rate);
    return this;
  }

  /**
   * Sets the speed of the motor.
   *
   * @param speed the speed to set the motor to, between -1.0 and 1.0
   */
  @Override
  public void set(double speed) {
    _motor.set(speed);
  }

  /**
   * Retrieves the current value of the motor.
   *
   * @return the current value of the motor, between -1.0 and 1.0
   */
  @Override
  public double get() {
    return _motor.get();
  }

  /**
   * Sets the inversion of the motor.
   *
   * @param isInverted a boolean indicating whether the motor should be inverted
   *                   or not
   */
  @Override
  public void setInverted(boolean isInverted) {
    _motor.setInverted(isInverted);
  }

  /**
   * Returns the inversion status of the motor.
   *
   * @return true if the motor is inverted, false otherwise
   */
  @Override
  public boolean getInverted() {
    return _motor.getInverted();
  }

  /**
   * Disables the motor by calling the disable method on the underlying motor
   * object.
   *
   * @see MotorController5010#disable()
   */
  @Override
  public void disable() {
    _motor.disable();
  }

  /**
   * Stops the motor by calling the stopMotor method on the underlying motor
   * object.
   *
   * @see MotorController5010#stopMotor()
   */
  @Override
  public void stopMotor() {
    _motor.stopMotor();
  }

  /**
   * Sets the motor as a follower of another motor.
   *
   * @param motor the motor to follow
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor) {
    _motor.setFollow(motor);
    return this;
  }

  /**
   * Sets the motor as a follower of another motor, with the option to invert the
   * follower.
   *
   * @param motor    the motor to follow
   * @param inverted whether the follower should be inverted or not
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
    _motor.setFollow(motor, inverted);
    return this;
  }

  /**
   * Inverts the motor.
   *
   * @param inverted whether the motor should be inverted or not
   * @return a reference to the current MotorController5010 instance
   */
  @Override
  public MotorController5010 invert(boolean inverted) {
    _motor.invert(inverted);
    return this;
  }

  /**
   * Retrieves the generic encoder for the motor.
   *
   * @return the generic encoder for the motor
   */
  @Override
  public GenericEncoder getMotorEncoder() {
    return _motor.getMotorEncoder();
  }

  /**
   * Retrieves the generic encoder for the motor.
   *
   * @param countsPerRev the number of counts per revolution of the motor
   * @return the generic encoder for the motor
   */
  @Override
  public GenericEncoder getMotorEncoder(int countsPerRev) {
    return _motor.getMotorEncoder(countsPerRev);
  }

  /**
   * Retrieves the PIDController5010 for this motor.
   *
   * @return the PIDController5010 for this motor
   * @throws UnsupportedOperationException if the motor does not support
   *                                       PIDController5010
   */
  @Override
  public PIDController5010 getPIDController5010() {
    throw new UnsupportedOperationException("Not supported for this motor");
  }

  /**
   * Returns the default SysIdRoutine using the given subsystem.
   *
   * @param subsystemBase the subsystem for which to create the default
   *                      SysIdRoutine
   * @return the default SysIdRoutine for the given subsystem
   */
  @Override
  public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
    return _motor.getDefaultSysId(subsystemBase);
  }

  /**
   * Retrieves the motor controller associated with this instance.
   *
   * @return the motor controller associated with this instance
   */
  @Override
  public Object getMotor() {
    return _motor.getMotor();
  }

  public GenericFunctionalMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
    _visualizer = visualizer;
    _robotToMotor = robotToMotor;
    return this;
  }

  public Mechanism2d getVisualizer() {
    return _visualizer;
  }

  /**
   * Needs to be overridden by subclasses to draw the motor behavior on the
   * visualizer
   */
  public void draw() {
  }

  /**
   * Returns the pose of the motor relative to the robot's origin.
   *
   * @return the pose of the motor relative to the robot's origin
   */
  public Pose3d getRobotToMotor() {
    return _robotToMotor;
  }

  /**
   * Needs to be overridden by subclasses to update the motor behavior during
   * simulation
   */
  public void simulationUpdate() {
  }

  /** Needs to be overridden by subclasses to return the motor simulation type */
  @Override
  public DCMotor getMotorSimulationType() {
    throw new UnsupportedOperationException("Unimplemented method 'getMotorSimulationType'");
  }

  /**
   * Converts a given distance in the x-direction to a x-coordinate appropriate
   * for visualizing on a Mechanism2d.
   *
   * @param x the distance in the x-direction
   * @return the x-coordinate for visualizing on a Mechanism2d
   */
  public double getSimX(Distance x) {
    return x.in(Meters) * RobotConstantsDef.robotVisualH / 2.0
        + RobotConstantsDef.robotVisualH / 2.0;
  }

  /**
   * Converts a given distance in the y-direction to a y-coordinate appropriate
   * for visualizing on a Mechanism2d.
   *
   * @param y the distance in the y-direction
   * @return the y-coordinate for visualizing on a Mechanism2d
   */
  public double getSimY(Distance y) {
    return y.in(Meters);
  }

  /**
   * Retrieves the maximum angular velocity of the motor in rotations per minute
   *
   * @return the maximum angular velocity of the motor in rotations per minute
   */
  @Override
  public AngularVelocity getMaxRPM() {
    throw new UnsupportedOperationException("Unimplemented method 'getMaxRPM'");
  }

  /**
   * Sets the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   * @return This motor controller.
   */
  @Override
  public MotorController5010 setVoltageCompensation(double nominalVoltage) {
    _motor.setVoltageCompensation(nominalVoltage);
    return this;
  }

  /**
   * Clears any sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults() {
    _motor.clearStickyFaults();
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
    _motor.setMotorBrake(isBrakeMode);
    return this;
  }

  /**
   * Saves the motor controller's configuration to flash memory. This must be
   * called after setting the motor controller's configuration in order to
   * persist the changes after the robot is restarted.
   */
  @Override
  public void burnFlash() {
    _motor.burnFlash();
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage() {
    return _motor.getVoltage();
  }

  /**
   * Get the applied output of the motor controller. This is the output as a
   * value from -1 to 1 that is actually being applied to the motor.
   *
   * @return The applied output of the motor.
   */
  @Override
  public double getAppliedOutput() {
    return _motor.getAppliedOutput();
  }

  @Override
  public double getOutputCurrent() {
    return _motor.getOutputCurrent();
  }

  /**
   * Resets the motor controller to its factory default settings.
   */
  @Override
  public void factoryDefaults() {
    _motor.factoryDefaults();
  }

  /**
   * Sets the DisplayValuesHelper that this motor will use to display
   * information on the dashboard. If this is not set, the motor will not be
   * visible on the dashboard.
   *
   * @param displayValuesHelper the display values helper to use
   */
  public void setDisplayValuesHelper(DisplayValuesHelper displayValuesHelper) {
    _displayValuesHelper = displayValuesHelper;
    _displayValuesHelper.nextColumn(_visualName);
    initiateDisplayValues();
  }

  /**
   * Initializes the display values for the motor. This method should be
   * implemented to set up the display values on the dashboard using the
   * DisplayValuesHelper. It is called after setting the DisplayValuesHelper
   * with {@link #setDisplayValuesHelper(DisplayValuesHelper)}.
   */
  protected void initiateDisplayValues() {
    throw new UnsupportedOperationException("Unimplemented method 'initiateDisplayValues'");
  }

  /**
   * Sets the simulated instance of the motor for use in simulations. This
   * method passes the motor simulation type to the motor controller, which
   * should be implemented to use the simulated motor type in its simulation.
   *
   * @param motorSimulationType the simulated instance of the motor
   */
  @Override
  public void setMotorSimulationType(DCMotor motorSimulationType) {
    _motor.setMotorSimulationType(motorSimulationType);
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
    _motor.simulationUpdate(position, velocity);
  }

  /**
   * Sets the maximum angular velocity of the motor in rotations per minute.
   *
   * @param rpm The maximum angular velocity of the motor in rotations per minute.
   */
  @Override
  public void setMaxRPM(AngularVelocity rpm) {
    _motor.setMaxRPM(rpm);
  }
}
