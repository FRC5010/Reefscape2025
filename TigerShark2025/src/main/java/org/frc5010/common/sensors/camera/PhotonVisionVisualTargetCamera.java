// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;

/** A camera using the PhotonVision library. */
public class PhotonVisionVisualTargetCamera extends PhotonVisionCamera {
  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param cameraToRobot - the camera-to-robot transform
   */
  public PhotonVisionVisualTargetCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super(name, colIndex, cameraToRobot);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.update();
    if (camResult.hasTargets()) {
      target = Optional.ofNullable(camResult.getBestTarget());
    }
  }
}
