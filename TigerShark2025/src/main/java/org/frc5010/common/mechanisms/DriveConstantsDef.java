// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.mechanisms;

/** Drive Train Constants that will be re-used year to year */
public class DriveConstantsDef {
  // ****** Persisted value name strings - use ALL_CAPS_UNDERSCORE_SEPARATED style
  // For consistency, make the String value the same as the Persisted variable below
  // This defines how it will display in the dashboards, so inconsistent naming will be confusing
  /** The velocity factor in volt-seconds per meter */
  public static final String KV_DRIVE_LINEAR = "kvVoltSecondsPerMeter";
  /** The accelleration factor in volts seconds squared per meter */
  public static final String KA_DRIVE_LINEAR = "kaVoltSecondsSquaredPerMeter";
  /** The motor rotations per wheel rotation */
  public static final String MOTOR_ROT_PER_WHEEL_ROT = "motorRotationsPerWheelRotation";
  /** The track width */
  public static final String TRACK_WIDTH = "trackWidth";
  /** The max chassis velocity */
  public static final String MAX_CHASSIS_VELOCITY = "maxChassisVelocity";
  /** The max chassis rotation rate */
  public static final String MAX_CHASSIS_ROTATION = "maxChassisRotation";

  /** The velocity factor in volt-seconds per radian */
  public static final String KV_DRIVE_ANGULAR = "kVAngular";
  /** The accelleration factor in volts seconds squared per radian */
  public static final String KA_DRIVE_ANGULAR = "kAAngular";

  /** The swerve turn proportional constant */
  public static final String SWERVE_TURN_P = "SwerveTurnP";
  /** The swerve turn integral constant */
  public static final String SWERVE_TURN_I = "SwerveTurnI";
  /** The swerve turn derivative constant */
  public static final String SWERVE_TURN_D = "SwerveTurnD";
}
