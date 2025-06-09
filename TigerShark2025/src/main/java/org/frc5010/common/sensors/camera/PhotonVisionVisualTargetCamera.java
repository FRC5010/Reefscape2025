// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** A camera using the PhotonVision library. */
public class PhotonVisionVisualTargetCamera extends PhotonVisionCamera {
  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param cameraToRobot - the camera-to-robot transform
   */
  public PhotonVisionVisualTargetCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super(name, colIndex, cameraToRobot);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target = Optional.ofNullable(camResult.getBestTarget());
      input.hasTarget = target.isPresent();
      input.latestTargetRotation = target
          .map(it -> new TargetRotation(
              new Rotation3d(0, target.get().getPitch(), target.get().getYaw())))
          .orElse(new TargetRotation(new Rotation3d()));
    }
  }
}
