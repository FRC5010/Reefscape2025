// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** A simulated camera using the PhotonVision library. */
public class SimulatedFiducialTargetCamera extends SimulatedCamera {
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();

  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param fieldLayout   - the field layout
   * @param strategy      - the pose strategy
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier  - the pose supplier
   * @param fiducialIds   - the list of fiducial IDs
   * @param width         - the camera width
   * @param height        - the camera height
   * @param fov           - the camera field of view
   */
  public SimulatedFiducialTargetCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds, int width, int height, double fov) {
    super(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier, width, height, fov);
    this.fiducialIds = fiducialIds;
    visionLayout.addDouble("Target ID", () -> target.map(it -> it.getFiducialId()).orElse(-1));
  }

  /** Update the simulated camera */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target = camResult.getTargets().stream()
          .filter(it -> fiducialIds.contains(it.getFiducialId()))
          .findFirst();
    }
  }

  @Override
  public boolean hasValidTarget() {
    return target.isPresent();
  }
}
