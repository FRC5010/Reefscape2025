// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;

/** A camera using the PhotonVision library. */
public class PhotonVisionFiducialTargetCamera extends PhotonVisionCamera {
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param fieldLayout - the field layout
   * @param cameraToRobot - the camera-to-robot transform
   * @param fiducialIds - the list of fiducial IDs
   */
  public PhotonVisionFiducialTargetCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      List<Integer> fiducialIds) {
    super(name, colIndex, cameraToRobot);
    this.fieldLayout = fieldLayout;
    this.fiducialIds = fiducialIds;
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target =
          camResult.getTargets().stream()
              .filter(it -> fiducialIds.contains(it.getFiducialId()))
              .findFirst();
    }
  }
}
