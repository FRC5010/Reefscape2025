// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class SwerveConstants extends GenericDrivetrainConstants {
  // public variables

  private SwerveDriveKinematics kinematics;

  private double kFrontLeftAbsoluteOffsetRad;
  private double kFrontRightAbsoluteOffsetRad;
  private double kBackLeftAbsoluteOffsetRad;
  private double kBackRightAbsoluteOffsetRad;

  private SwerveModuleConstants swerveModuleConstants;

  public SwerveConstants(Distance trackWidth, Distance wheelBase) {
    this.DRIVETRAIN_TRACKWIDTH = trackWidth;
    this.DRIVETRAIN_WHEELBASE = wheelBase;

    kinematics =
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH.in(Meters) / 2.0, DRIVETRAIN_WHEELBASE.in(Meters) / 2.0),
            // Front right
            new Translation2d(
                DRIVETRAIN_TRACKWIDTH.in(Meters) / 2.0, -DRIVETRAIN_WHEELBASE.in(Meters) / 2.0),
            // Back left
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH.in(Meters) / 2.0, DRIVETRAIN_WHEELBASE.in(Meters) / 2.0),
            // Back right
            new Translation2d(
                -DRIVETRAIN_TRACKWIDTH.in(Meters) / 2.0, -DRIVETRAIN_WHEELBASE.in(Meters) / 2.0));
  }

  public SwerveConstants(GenericDrivetrainConstants constants) {
    super(constants);
    swerveModuleConstants = new SwerveModuleConstants();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void configureSwerve(double motorMaxRPM, double turnMotorMaxRPM) {
    kPhysicalMaxSpeedMetersPerSecond =
        motorMaxRPM * swerveModuleConstants.getkDriveEncoderRPM2MeterPerSec();
    kPhysicalMaxAngularSpeedRadiansPerSecond =
        turnMotorMaxRPM * swerveModuleConstants.getkTurningEncoderRPM2RadPerSec();
  }

  public double getkFrontLeftAbsoluteOffsetRad() {
    return kFrontLeftAbsoluteOffsetRad;
  }

  public void setkFrontLeftAbsoluteOffsetRad(double kFrontLeftAbsoluteOffsetRad) {
    this.kFrontLeftAbsoluteOffsetRad = kFrontLeftAbsoluteOffsetRad;
  }

  public double getkFrontRightAbsoluteOffsetRad() {
    return kFrontRightAbsoluteOffsetRad;
  }

  public void setkFrontRightAbsoluteOffsetRad(double kFrontRightAbsoluteOffsetRad) {
    this.kFrontRightAbsoluteOffsetRad = kFrontRightAbsoluteOffsetRad;
  }

  public double getkBackLeftAbsoluteOffsetRad() {
    return kBackLeftAbsoluteOffsetRad;
  }

  public void setkBackLeftAbsoluteOffsetRad(double kBackLeftAbsoluteOffsetRad) {
    this.kBackLeftAbsoluteOffsetRad = kBackLeftAbsoluteOffsetRad;
  }

  public double getkBackRightAbsoluteOffsetRad() {
    return kBackRightAbsoluteOffsetRad;
  }

  public void setkBackRightAbsoluteOffsetRad(double kBackRightAbsoluteOffsetRad) {
    this.kBackRightAbsoluteOffsetRad = kBackRightAbsoluteOffsetRad;
  }

  public SwerveModuleConstants getSwerveModuleConstants() {
    return swerveModuleConstants;
  }

  public void setSwerveModuleConstants(SwerveModuleConstants swerveModuleConstants) {
    this.swerveModuleConstants = swerveModuleConstants;
  }
}
