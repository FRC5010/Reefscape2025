package org.frc5010.common.drive.swerve_utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Kinematic path following constraints
 *
 * @param maxVelocityMPS                    Max linear velocity (M/S)
 * @param maxAccelerationMPSSq              Max linear acceleration (M/S^2)
 * @param maxAngularVelocityRadPerSec       Max angular velocity (Rad/S)
 * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
 * @param nominalVoltageVolts               The nominal battery voltage (Volts)
 * @param unlimited                         Should the constraints be unlimited
 */
public record PathConstraints5010(
    double maxVelocityMPS,
    LinearAcceleration maxPhysicalAcceleration,
    Supplier<LinearAcceleration> maxForwardAccelerationMPSSq,
    Supplier<LinearAcceleration> maxBackwardAccelerationMPSSq,
    Supplier<LinearAcceleration> maxLeftAccelerationMPSSq,
    Supplier<LinearAcceleration> maxRightAccelerationMPSSq,
    Supplier<LinearVelocity> maxForwardVelocity,
    Supplier<LinearVelocity> maxBackwardVelocity,
    Supplier<LinearVelocity> maxLeftVelocity,
    Supplier<LinearVelocity> maxRightVelocity,
    double maxAngularVelocityRadPerSec,
    double maxAngularAccelerationRadPerSecSq,
    double nominalVoltageVolts,
    boolean unlimited) {
  /**
   * Kinematic path following constraints
   *
   * @param maxVelocity            Max linear velocity
   * @param maxAcceleration        Max linear acceleration
   * @param maxAngularVelocity     Max angular velocity
   * @param maxAngularAcceleration Max angular acceleration
   * @param nominalVoltage         The nominal battery voltage
   * @param unlimited              Should the constraints be unlimited
   */
  public PathConstraints5010(
      LinearVelocity maxVelocity,
      LinearAcceleration maxPhysicalAcceleration,
      Supplier<LinearAcceleration> maxForwardAcceleration,
      Supplier<LinearAcceleration> maxBackwardAcceleration,
      Supplier<LinearAcceleration> maxLeftAcceleration,
      Supplier<LinearAcceleration> maxRightAcceleration,
      Supplier<LinearVelocity> maxForwardVelocity,
      Supplier<LinearVelocity> maxBackwardVelocity,
      Supplier<LinearVelocity> maxLeftVelocity,
      Supplier<LinearVelocity> maxRightVelocity,
      AngularVelocity maxAngularVelocity,
      AngularAcceleration maxAngularAcceleration,
      Voltage nominalVoltage,
      boolean unlimited) {
    this(
        maxVelocity.in(MetersPerSecond),
        maxPhysicalAcceleration,
        maxForwardAcceleration,
        maxBackwardAcceleration,
        maxLeftAcceleration,
        maxRightAcceleration,
        maxForwardVelocity,
        maxBackwardVelocity,
        maxLeftVelocity,
        maxRightVelocity,
        maxAngularVelocity.in(RadiansPerSecond),
        maxAngularAcceleration.in(RadiansPerSecondPerSecond),
        nominalVoltage.in(Volts),
        unlimited);
  }

  /**
   * Get the max linear velocity
   *
   * @return Max linear velocity
   */
  public LinearVelocity maxVelocity() {
    return MetersPerSecond.of(maxVelocityMPS);
  }

  /**
   * Get the max linear acceleration
   *
   * @return Max linear acceleration
   */
  public double maxLinearAcceleration() {
    return maxPhysicalAcceleration.in(MetersPerSecondPerSecond);
  }

  public double maxForwardAcceleration() {
    return maxForwardAccelerationMPSSq.get().in(MetersPerSecondPerSecond);
  }

  public double maxBackwardAcceleration() {
    return maxBackwardAccelerationMPSSq.get().in(MetersPerSecondPerSecond);
  }

  public double maxRightAcceleration() {
    return maxRightAccelerationMPSSq.get().in(MetersPerSecondPerSecond);
  }

  public double maxLeftAcceleration() {
    return maxLeftAccelerationMPSSq.get().in(MetersPerSecondPerSecond);
  }

  public double getMaxForwardVelocity() {
    return maxForwardVelocity.get().in(MetersPerSecond);
  }

  public double getMaxBackwardVelocity() {
    return maxBackwardVelocity.get().in(MetersPerSecond);
  }

  public double getMaxRightVelocity() {
    return maxRightVelocity.get().in(MetersPerSecond);
  }

  public double getMaxLeftVelocity() {
    return maxLeftVelocity.get().in(MetersPerSecond);
  }

  /**
   * Get the max angular velocity
   *
   * @return Max angular velocity
   */
  public AngularVelocity maxAngularVelocity() {
    return RadiansPerSecond.of(maxAngularVelocityRadPerSec);
  }

  /**
   * Get the max angular acceleration
   *
   * @return Max angular acceleration
   */
  public AngularAcceleration maxAngularAcceleration() {
    return RadiansPerSecondPerSecond.of(maxAngularAccelerationRadPerSecSq);
  }
}