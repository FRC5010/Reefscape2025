// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.function.DriveTrainMotor;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.hardware.GenericRevBrushlessMotor;
import org.frc5010.common.motors.hardware.GenericThriftyNovaMotor;
import org.frc5010.common.motors.hardware.KrakenX60;

import edu.wpi.first.units.measure.Current;
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

  public static MotorController5010 Neo(int port) {
    return Neo(port, MotorConstants.Motor.Neo.currentLimit);
  }

  public static MotorController5010 Neo(int port, Current currentLimit) {
    MotorController5010 motor = new GenericRevBrushlessMotor(port, currentLimit);
    motor.setMaxRPM(MotorConstants.Motor.Neo.maxRpm);
    motor.setMotorSimulationType(MotorConstants.Motor.Neo.motorSim);
    return motor;
  }

  public static MotorController5010 Spark(int canId, Motor config) {
    return new GenericRevBrushlessMotor(canId, config);
  }

  public static MotorController5010 Neo550(int port) {
    return Neo550(port, MotorConstants.Motor.Neo550.currentLimit);
  }

  public static MotorController5010 Neo550(int port, Current currentLimit) {
    MotorController5010 motor = new GenericRevBrushlessMotor(port, currentLimit);
    motor.setMaxRPM(MotorConstants.Motor.Neo550.maxRpm);
    motor.setMotorSimulationType(MotorConstants.Motor.Neo550.motorSim);
    return motor;
  }

  public static MotorController5010 ThriftyNeo(int port) {
    return ThriftyNeo(port, MotorConstants.Motor.Neo.currentLimit);
  }

  public static MotorController5010 ThriftyNeo(int port, Current currentLimit) {
    MotorController5010 motor = new GenericThriftyNovaMotor(port, currentLimit);
    motor.setMaxRPM(MotorConstants.Motor.Neo.maxRpm);
    motor.setMotorSimulationType(MotorConstants.Motor.Neo.motorSim);
    return motor;
  }

  public static MotorController5010 ThriftyNeo550(int port) {
    return ThriftyNeo550(port, MotorConstants.Motor.Neo550.currentLimit);
  }

  public static MotorController5010 ThriftyNeo550(int port, Current currentLimit) {
    MotorController5010 motor = new GenericThriftyNovaMotor(port, currentLimit);
    motor.setMaxRPM(MotorConstants.Motor.Neo550.maxRpm);
    motor.setMotorSimulationType(Motor.Neo550.motorSim);
    return motor;
  }

  public static MotorController5010 Thrifty(int canId, Motor config) {
    return new GenericThriftyNovaMotor(canId, config);
  }

  public static MotorController5010 KrakenX60(int port) {
    return KrakenX60(port, Motor.KrakenX60.currentLimit);
  }

  public static MotorController5010 KrakenX60(int port, Current currentLimit) {
    MotorController5010 motor = new KrakenX60(port);
    motor.setMaxRPM(Motor.KrakenX60.maxRpm);
    motor.setMotorSimulationType(Motor.KrakenX60.motorSim);
    return motor;
  }

  public static MotorController5010 DriveTrainMotor(MotorController5010 motor, String name) {
    return new DriveTrainMotor(motor, name);
  }

  public static MotorController5010 FollowMotor(
      MotorController5010 motor, MotorController5010 leader) {
    return new FollowerMotor(motor, leader, "");
  }
}
