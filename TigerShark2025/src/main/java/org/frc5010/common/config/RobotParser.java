// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.json.CameraConfigurationJson;
import org.frc5010.common.config.json.DriveteamControllersJson;
import org.frc5010.common.config.json.DrivetrainPropertiesJson;
import org.frc5010.common.config.json.RobotJson;
import org.frc5010.common.config.json.VisionPropertiesJson;
import org.frc5010.common.config.json.YAGSLDrivetrainJson;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/** RobotParser is used to parse JSON configuration files to build a robot. */
public class RobotParser {
  /** JSON classes for the Driveteam controllers */
  private static DriveteamControllersJson controllersJson;
  /** Map of Driveteam controller configurations */
  private static Map<String, DriveteamControllerConfiguration> controllersMap;
  /** JSON classes for the cameras */
  private static VisionPropertiesJson visionJson;
  /** Map of camera configurations */
  private static Map<String, CameraConfigurationJson> camerasMap;
  /** JSON class for the drivetrain */
  private static Optional<DrivetrainPropertiesJson> driveTrainJson = Optional.empty();

  /**
   * Creates a new RobotParser.
   *
   * @param robotDirectory the directory to read from
   * @param robot the robot being configured
   * @throws IOException
   */
  public RobotParser(String robotDirectory, GenericRobot robot) throws IOException {
    File directory = new File(Filesystem.getDeployDirectory(), robotDirectory);
    checkDirectory(directory);

    // Read in the robot configuration
    RobotJson robotJson =
        new ObjectMapper().readValue(new File(directory, "robot.json"), RobotJson.class);
    robotJson.configureRobot(robot, directory);

    // Read in the controllers
    controllersJson =
        new ObjectMapper()
            .readValue(new File(directory, "controllers.json"), DriveteamControllersJson.class);
    controllersMap = controllersJson.readControllers(directory);

    // Read in the cameras
    visionJson =
        new ObjectMapper()
            .readValue(new File(directory, "cameras.json"), VisionPropertiesJson.class);
    camerasMap = visionJson.readCameraSystem(directory);

    switch (robotJson.driveType) {
      case "YAGSL_SWERVE_DRIVE":
        {
          YAGSLDrivetrainJson yagslDriveTrainJson =
              new ObjectMapper()
                  .readValue(
                      new File(directory, "yagsl_drivetrain.json"), YAGSLDrivetrainJson.class);
          yagslDriveTrainJson.readDrivetrainConfiguration(robot, directory);
          driveTrainJson = Optional.of(yagslDriveTrainJson);
          break;
        }
      default:
        break;
    }
  }

  /**
   * Method to check the existence of specific JSON configuration files in the provided directory.
   *
   * @param directory the directory to check for JSON configuration files
   */
  private void checkDirectory(File directory) {
    assert new File(directory, "robot.json").exists();
  }

  /**
   * Method to create the robot.
   *
   * @param robot description of parameter
   */
  public void createRobot(GenericRobot robot) {
    controllersJson.createControllers(robot, controllersMap);
    driveTrainJson.ifPresent(it -> it.createDriveTrain(robot));
    visionJson.createCameraSystem(robot, camerasMap);
  }
}
