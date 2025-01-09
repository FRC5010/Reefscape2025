// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

/** JSON class with an array of cameras to configure */
public class VisionPropertiesJson {
  /** An array of camera names */
  public String[] cameras;

  /**
   * Creates cameras for a given robot using the provided map of camera
   * configurations.
   *
   * @param robot the robot to add the cameras to
   * @param map   a map of camera configurations, where the key is the camera name
   *              and the value is
   *              the configuration object
   */
  public void createCameraSystem(GenericRobot robot, Map<String, CameraConfigurationJson> map) {
    map.keySet()
        .forEach(
            it -> {
              map.get(it).configureCamera(robot);
            });
  }

  /**
   * Reads in cameras from the provided directory.
   *
   * @param directory the directory to read from
   * @return the map of camera configurations
   */
  public Map<String, CameraConfigurationJson> readCameraSystem(File directory)
      throws IOException, StreamReadException, DatabindException {
    Map<String, CameraConfigurationJson> camerasMap = new HashMap<>();
    for (int i = 0; i < cameras.length; i++) {
      File cameraFile = new File(directory, "cameras/" + cameras[i]);
      assert cameraFile.exists();
      CameraConfigurationJson camera = new ObjectMapper().readValue(cameraFile, CameraConfigurationJson.class);
      camerasMap.put(camera.name, camera);
    }
    return camerasMap;
  }
}
