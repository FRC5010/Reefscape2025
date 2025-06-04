// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import org.frc5010.common.arch.GenericDeviceHandler;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.constants.GenericDrivetrainConstants;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

/** The base JSON class for robot configurations */
public class RobotJson {
  /* User configuration file for the power mode */
  public String userConfig = "competitionMode.json";

  /** Drivetrain configuration */
  public String driveType = "YAGSL_SWERVE_DRIVE";

  /** Drivetrain width between wheel centers */
  public double trackWidth = 0.0;

  /** Drivetrain width units */
  public String trackWidthUom = "m";

  /** Drivetrain length between wheel centers */
  public double wheelBase = 0.0;

  /** Drivetrain length units */
  public String wheelBaseUom = "m";

  /** Drivetrain wheel diameter */
  public double wheelDiameter = 0.0;

  /** Drivetrain wheel diameter units */
  public String wheelDiameterUom = "m";

  /** Drivetrain physical max speed */
  public double physicalMaxSpeed = 0.0;

  /** Drivetrain physical max speed units */
  public String physicalMaxSpeedUom = "m/s";

  /** Drivetrain gear ratio between drive motor and wheels */
  public double driveMotorGearRatio = 1.0;

  /** Game piece definition for first game piece */
  public String gamePieceA = "GPA";

  /** Game piece definition for second game piece */
  public String gamePieceB = "GPB";

  /** Device definition files */
  public Map<String, String> devices;

  /**
   * Reads the robot configuration from the given directory
   *
   * @param robot the robot being configured
   * @param directory the directory to read from
   * @throws IOException
   */
  public void configureRobot(GenericRobot robot, File directory) throws IOException {
    GenericDrivetrainConstants drivetrainConstants = robot.getDrivetrainConstants();
    drivetrainConstants.setTrackWidth(UnitsParser.parseDistance(trackWidth, trackWidthUom));
    drivetrainConstants.setWheelBase(UnitsParser.parseDistance(wheelBase, wheelBaseUom));
    drivetrainConstants.setWheelDiameter(wheelDiameter);
    drivetrainConstants.setkPhysicalMaxSpeedMetersPerSecond(physicalMaxSpeed);

    UserModeJson userModeJson =
        new ObjectMapper().readValue(new File(directory, userConfig), UserModeJson.class);
    drivetrainConstants.setkTeleDriveMaxSpeedMetersPerSecond(userModeJson.maxSpeed);
    drivetrainConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(userModeJson.maxAngularSpeed);
    drivetrainConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(userModeJson.maxAccelleration);
    drivetrainConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(
        userModeJson.maxAngularAccelleration);
    drivetrainConstants.setkDriveMotorGearRatio(driveMotorGearRatio);

    Constants.Simulation.gamePieceA = gamePieceA;
    Constants.Simulation.gamePieceB = gamePieceB;
  }

  /**
   * Reads the mechanism definition files
   *
   * @param system the system being configured
   * @throws IOException
   * @throws DatabindException
   * @throws StreamReadException
   */
  public void readDeviceDefinitions(GenericDeviceHandler system, File directory)
      throws StreamReadException, DatabindException, IOException {
    for (String key : devices.keySet()) {
      File mechanismDefFile = new File(directory, "devices/" + devices.get(key));
      assert mechanismDefFile.exists();
      DeviceConfigReader.readDeviceConfig(system, mechanismDefFile, key);
    }
  }
}
