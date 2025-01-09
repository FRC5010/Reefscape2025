// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import edu.wpi.first.wpilibj.util.Color;
import org.frc5010.common.motors.function.DriveTrainMotor;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.hardware.KrakenX60;
import org.frc5010.common.motors.hardware.NEO;
import org.frc5010.common.motors.hardware.NEO550;

/** Add your docs here. */
public class MotorFactory {
  protected static int simEncoderPort = 10;
  protected static int visualColorIndex = 0;
  protected static Color[] visualColors =
      new Color[] {
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

  public static MotorController5010 NEO(int port) {
    return new NEO(port);
  }

  public static MotorController5010 NEO(int port, int currentLimit) {
    return new NEO(port, currentLimit);
  }

  public static MotorController5010 NEO550(int port) {
    return new NEO550(port);
  }

  public static MotorController5010 NEO550(int port, int currentLimit) {
    return new NEO550(port, currentLimit);
  }

  public static MotorController5010 KrakenX60(int port) {
    return new KrakenX60(port);
  }

  public static MotorController5010 DriveTrainMotor(MotorController5010 motor, String name) {
    return new DriveTrainMotor(motor, name);
  }

  public static MotorController5010 FollowMotor(
      MotorController5010 motor, MotorController5010 leader) {
    return new FollowerMotor(motor, leader, "");
  }
}
