// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/** A camera using the PhotonVision library. */
public class PhotonVisionFiducialTargetCamera extends PhotonVisionPoseCamera {
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();

  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param fieldLayout   - the field layout
   * @param cameraToRobot - the camera-to-robot transform
   * @param fiducialIds   - the list of fiducial IDs
   */
  public PhotonVisionFiducialTargetCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds) {
    super(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier, fiducialIds);
    this.fieldLayout = fieldLayout;
    this.fiducialIds = fiducialIds;
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target = camResult.getTargets().stream()
          .filter(it -> fiducialIds.contains(it.getFiducialId()))
          .findFirst();

      input.hasTarget = target.isPresent();
      input.latestTargetRotation = target
          .map(it -> new TargetRotation(new Rotation3d(0, it.getPitch(), it.getYaw())))
          .orElse(new TargetRotation(new Rotation3d()));
      input.latestTargetPose = target
          .map(it -> {
            Transform3d targetTrans = robotToCamera.plus(it.getBestCameraToTarget());
            return new Pose3d(targetTrans.getTranslation(), targetTrans.getRotation());
          })
          .orElse(new Pose3d());
    }
  }
}
