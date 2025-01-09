// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Constants used to define drivetrains */
public class GenericDrivetrainConstants {
  /** The drive train track width in metere */
  protected Distance DRIVETRAIN_TRACKWIDTH;
  /** The drive train wheel base in meters */
  protected Distance DRIVETRAIN_WHEELBASE;
  /** The drive train wheel diameter in meters */
  protected double kWheelDiameterMeters; // = Units.inchesToMeters(3);
  /** The drive train gear ratio */
  protected double kDriveMotorGearRatio;

  /** Physical max angular speed in radians per second */
  protected double kPhysicalMaxAngularSpeedRadiansPerSecond;

  /** Physical max speed in meters per second */
  protected double kPhysicalMaxSpeedMetersPerSecond;

  /** Driver limited max speed in meters per second */
  protected double kTeleDriveMaxSpeedMetersPerSecond;

  /** Driver limited max angular speed in radians per second */
  protected double kTeleDriveMaxAngularSpeedRadiansPerSecond;

  /** Driver limited max acceleration in meters per second squared */
  protected double kTeleDriveMaxAccelerationUnitsPerSecond;

  /** Driver limited max angular acceleration in radians per second squared */
  protected double kTeleDriveMaxAngularAccelerationUnitsPerSecond;

  /** Instantiates a new GenericDrivetrainConstants */
  public GenericDrivetrainConstants() {
    this.DRIVETRAIN_TRACKWIDTH = Meters.of(0.0);
    this.DRIVETRAIN_WHEELBASE = Meters.of(0.0);
    this.kWheelDiameterMeters = 0.0;
    this.kDriveMotorGearRatio = 0.0;
    this.kPhysicalMaxAngularSpeedRadiansPerSecond = 0.0;
    this.kPhysicalMaxSpeedMetersPerSecond = 0.0;
    this.kTeleDriveMaxSpeedMetersPerSecond = 0.0;
    this.kTeleDriveMaxAngularSpeedRadiansPerSecond = 0.0;
    this.kTeleDriveMaxAccelerationUnitsPerSecond = 0.0;
    this.kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.0;
  }

  /**
   * Instantiates a new GenericDrivetrainConstants from another GenericDrivetrainConstants
   *
   * @param constants the other GenericDrivetrainConstants
   */
  public GenericDrivetrainConstants(GenericDrivetrainConstants constants) {
    this.DRIVETRAIN_TRACKWIDTH = constants.DRIVETRAIN_TRACKWIDTH;
    this.DRIVETRAIN_WHEELBASE = constants.DRIVETRAIN_WHEELBASE;
    this.kWheelDiameterMeters = constants.kWheelDiameterMeters;
    this.kDriveMotorGearRatio = constants.kDriveMotorGearRatio;
    this.kPhysicalMaxAngularSpeedRadiansPerSecond =
        constants.kPhysicalMaxAngularSpeedRadiansPerSecond;
    this.kPhysicalMaxSpeedMetersPerSecond = constants.kPhysicalMaxSpeedMetersPerSecond;
    this.kTeleDriveMaxSpeedMetersPerSecond = constants.kTeleDriveMaxSpeedMetersPerSecond;
    this.kTeleDriveMaxAngularSpeedRadiansPerSecond =
        constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    this.kTeleDriveMaxAccelerationUnitsPerSecond =
        constants.kTeleDriveMaxAccelerationUnitsPerSecond;
    this.kTeleDriveMaxAngularAccelerationUnitsPerSecond =
        constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }

  /**
   * Sets the track width of the drivetrain.
   *
   * @param trackWidth the new track width in meters
   */
  public void setTrackWidth(Distance trackWidth) {
    this.DRIVETRAIN_TRACKWIDTH = trackWidth;
  }

  /**
   * Returns the track width of the drivetrain in meters.
   *
   * @return the track width in meters
   */
  public Distance getTrackWidth() {
    return DRIVETRAIN_TRACKWIDTH;
  }

  /**
   * Sets the wheel base of the drivetrain in meters.
   *
   * @param wheelBase the new wheel base in meters
   */
  public void setWheelBase(Distance wheelBase) {
    this.DRIVETRAIN_WHEELBASE = wheelBase;
  }

  /**
   * Returns the wheel base of the drivetrain in meters.
   *
   * @return the wheel base in meters
   */
  public Distance getWheelBase() {
    return DRIVETRAIN_WHEELBASE;
  }

  /**
   * Sets the wheel diameter of the drivetrain in meters.
   *
   * @param wheelDiameter the new wheel diameter in meters
   */
  public void setWheelDiameter(double wheelDiameter) {
    this.kWheelDiameterMeters = wheelDiameter;
  }

  /**
   * Returns the wheel diameter of the drivetrain in meters.
   *
   * @return the wheel diameter in meters
   */
  public double getWheelDiameter() {
    return kWheelDiameterMeters;
  }

  /**
   * Returns the maximum angular speed of the drivetrain in radians per second.
   *
   * @return the maximum angular speed in radians per second
   */
  public double getkPhysicalMaxAngularSpeedRadiansPerSecond() {
    return kPhysicalMaxAngularSpeedRadiansPerSecond;
  }

  /**
   * Sets the maximum angular speed of the drivetrain in radians per second.
   *
   * @param kPhysicalMaxAngularSpeedRadiansPerSecond the new maximum angular speed in radians per
   *     second
   */
  public void setkPhysicalMaxAngularSpeedRadiansPerSecond(
      double kPhysicalMaxAngularSpeedRadiansPerSecond) {
    this.kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
  }

  /**
   * Retrieves the maximum physical speed of the drivetrain in meters per second.
   *
   * @return the maximum physical speed in meters per second
   */
  public double getkPhysicalMaxSpeedMetersPerSecond() {
    return kPhysicalMaxSpeedMetersPerSecond;
  }

  /**
   * Sets the maximum physical speed of the drivetrain in meters per second.
   *
   * @param kPhysicalMaxSpeedMetersPerSecond the new maximum physical speed in meters per second
   */
  public void setkPhysicalMaxSpeedMetersPerSecond(double kPhysicalMaxSpeedMetersPerSecond) {
    this.kPhysicalMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
  }

  /**
   * Retrieves the maximum speed in meters per second for teleoperated driving.
   *
   * @return the maximum speed in meters per second
   */
  public double getkTeleDriveMaxSpeedMetersPerSecond() {
    return kTeleDriveMaxSpeedMetersPerSecond;
  }

  /**
   * Sets the maximum speed in meters per second for teleoperated driving.
   *
   * @param kTeleDriveMaxSpeedMetersPerSecond the new maximum speed in meters per second
   */
  public void setkTeleDriveMaxSpeedMetersPerSecond(double kTeleDriveMaxSpeedMetersPerSecond) {
    this.kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecond;
  }

  /**
   * Retrieves the maximum angular speed in radians per second for teleoperated driving.
   *
   * @return the maximum angular speed in radians per second
   */
  public double getkTeleDriveMaxAngularSpeedRadiansPerSecond() {
    return kTeleDriveMaxAngularSpeedRadiansPerSecond;
  }

  /**
   * Sets the maximum angular speed in radians per second for teleoperated driving.
   *
   * @param kTeleDriveMaxAngularSpeedRadiansPerSecond the new maximum angular speed in radians per
   *     second
   */
  public void setkTeleDriveMaxAngularSpeedRadiansPerSecond(
      double kTeleDriveMaxAngularSpeedRadiansPerSecond) {
    this.kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxAngularSpeedRadiansPerSecond;
  }

  /**
   * Returns the maximum acceleration in units per second for teleoperated driving.
   *
   * @return the maximum acceleration in units per second
   */
  public double getkTeleDriveMaxAccelerationUnitsPerSecond() {
    return kTeleDriveMaxAccelerationUnitsPerSecond;
  }

  /**
   * Sets the maximum acceleration in units per second for teleoperated driving.
   *
   * @param kTeleDriveMaxAccelerationUnitsPerSecond the new maximum acceleration in units per second
   *     for teleoperated driving
   */
  public void setkTeleDriveMaxAccelerationUnitsPerSecond(
      double kTeleDriveMaxAccelerationUnitsPerSecond) {
    this.kTeleDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond;
  }

  /**
   * Retrieves the maximum angular acceleration in units per second for teleoperated driving.
   *
   * @return the maximum angular acceleration in units per second
   */
  public double getkTeleDriveMaxAngularAccelerationUnitsPerSecond() {
    return kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }

  /**
   * Sets the maximum angular acceleration in units per second for teleoperated driving.
   *
   * @param kTeleDriveMaxAngularAccelerationUnitsPerSecond the new maximum angular acceleration in
   *     units per second for teleoperated driving
   */
  public void setkTeleDriveMaxAngularAccelerationUnitsPerSecond(
      double kTeleDriveMaxAngularAccelerationUnitsPerSecond) {
    this.kTeleDriveMaxAngularAccelerationUnitsPerSecond =
        kTeleDriveMaxAngularAccelerationUnitsPerSecond;
  }

  /**
   * Returns the gear ratio of the drive motor.
   *
   * @return the gear ratio of the drive motor
   */
  public double getkDriveMotorGearRatio() {
    return kDriveMotorGearRatio;
  }

  /**
   * Sets the gear ratio of the drive motor.
   *
   * @param kDriveMotorGearRatio the new gear ratio of the drive motor
   */
  public void setkDriveMotorGearRatio(double kDriveMotorGearRatio) {
    this.kDriveMotorGearRatio = kDriveMotorGearRatio;
  }
}
