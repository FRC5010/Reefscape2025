// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.function.DriveTrainMotor;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;
import org.frc5010.common.motors.hardware.GenericThriftyNovaMotor;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class MotorFactory {
  protected static int simEncoderPort = 10;
  protected static int visualColorIndex = 0;
  protected static Color[] visualColors = new Color[] {
      Color.kRed,
      Color.kOrange,
      Color.kYellow,
      Color.kGreen,
      Color.kBlue,
      Color.kPurple,
      Color.kCyan,
      Color.kMagenta,
      Color.kViolet,
      Color.kPink,
      Color.kWhite,
      Color.kBrown,
      Color.kDarkRed,
      Color.kDarkOrange,
      Color.kYellowGreen,
      Color.kDarkGreen,
      Color.kDarkBlue,
      Color.kDarkViolet,
      Color.kDarkCyan,
      Color.kDarkMagenta,
      Color.kDarkSalmon,
      Color.kGray
  };

  public static int getNextSimEncoderPort() {
    return simEncoderPort++;
  }

  public static Color getNextVisualColor() {
    if (visualColorIndex >= visualColors.length) {
      visualColorIndex = 0;
    }
    return visualColors[visualColorIndex++];
  }

  public static MotorController5010 Spark(int canId, Motor config) {
    switch (config) {
      case KrakenX60:
        throw new IllegalArgumentException("Sparks can not use KrakenX60 config");
      default:
    }
    return new GenericRevBrushlessMotor(canId, config);
  }

  public static MotorController5010 Thrifty(int canId, Motor config) {
    switch (config) {
      case KrakenX60:
        throw new IllegalArgumentException("Thrifty Novas can not use KrakenX60 config");
      default:
    }
    return new GenericThriftyNovaMotor(canId, config);
  }

  public static MotorController5010 TalonFX(int canId, Motor config) {
    switch (config) {
      case KrakenX60:
      return new GenericTalonFXMotor(canId, config);
      default:
      throw new IllegalArgumentException("TalonFX can not use " + config + " config");
    }
  }

  public static MotorController5010 DriveTrainMotor(MotorController5010 motor, String name) {
    return new DriveTrainMotor(motor, name);
  }

  public static MotorController5010 FollowMotor(
      MotorController5010 motor, MotorController5010 leader) {
    return new FollowerMotor(motor, leader, "");
  }
}
