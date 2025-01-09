// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** A camera using the PhotonVision library. */
public class PhotonVisionPoseCamera extends PhotonVisionCamera {
  /** The pose estimator */
  protected PhotonPoseEstimator poseEstimator;
  /** The pose strategy */
  protected PoseStrategy strategy;
  /** The pose supplier */
  protected Supplier<Pose2d> poseSupplier;

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param fieldLayout - the field layout
   * @param strategy - the pose strategy
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier - the pose supplier
   */
  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier) {
    super(name, colIndex, cameraToRobot);
    this.strategy = strategy;
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;
    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, cameraToRobot);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target = Optional.ofNullable(camResult.getBestTarget());
    }
  }

  /**
   * Get the target pose estimate relative to the robot.
   *
   * @return the target pose estimate relative to the robot
   */
  @Override
  public Optional<Pose3d> getRobotPose() {
    Pose3d robotPoseEst = null;
    if (target.isPresent()) {
      Optional<EstimatedRobotPose> result = poseEstimator.update(camResult);

      if (result.isPresent()
          && result.get().estimatedPose != null
          && target.get().getPoseAmbiguity() < 0.5) {
        robotPoseEst = result.get().estimatedPose;
      }
    }
    return Optional.ofNullable(robotPoseEst);
  }

  /**
   * Get the target pose estimate relative to the robot.
   *
   * @return the target pose estimate relative to the robot
   */
  @Override
  public Optional<Pose3d> getRobotToTargetPose() {
    Pose3d targetPoseEst = null;
    if (target.isPresent()) {
      if (target.get().getFiducialId() != 0) {
        Transform3d robotToTarget = robotToCamera.plus(target.get().getBestCameraToTarget());
        targetPoseEst = new Pose3d(robotToTarget.getTranslation(), robotToTarget.getRotation());
      }
    }
    return Optional.ofNullable(targetPoseEst);
  }
}
