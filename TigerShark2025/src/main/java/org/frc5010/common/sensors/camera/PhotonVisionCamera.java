// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** A camera using the PhotonVision library. */
public class PhotonVisionCamera extends GenericCamera {
  /** The camera */
  protected PhotonCamera camera;
  /** The field layout */
  protected AprilTagFieldLayout fieldLayout;
  /** The target, if any */
  protected Optional<PhotonTrackedTarget> target = Optional.empty();
  /** The latest camera result */
  protected PhotonPipelineResult camResult;
  /** The latest camera results */
  protected List<PhotonPipelineResult> camResults;

  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param cameraToRobot - the camera-to-robot transform
   */
  public PhotonVisionCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super(name, colIndex, cameraToRobot);
    this.robotToCamera = cameraToRobot;
    camera = new PhotonCamera(name);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    camResults = camera.getAllUnreadResults();
    camResult = camResults.stream().findFirst().orElse(new PhotonPipelineResult());
  }

  /**
   * Does the camera have a valid target?
   *
   * @return true if the camera has a valid target
   */
  @Override
  public boolean hasValidTarget() {
    return camResult.hasTargets();
  }

  /**
   * Get the target yaw
   *
   * @return the target yaw
   */
  @Override
  public double getTargetYaw() {
    return target.map(t -> t.getYaw()).orElse(Double.MAX_VALUE);
  }

  /**
   * Get the target pitch
   *
   * @return the target pitch
   */
  @Override
  public double getTargetPitch() {
    return target.map(t -> t.getPitch()).orElse(Double.MAX_VALUE);
  }

  /**
   * Get the target area
   *
   * @return the target area
   */
  @Override
  public double getTargetArea() {
    return target.map(t -> t.getArea()).orElse(Double.MAX_VALUE);
  }

  /**
   * Get the latency in seconds
   *
   * @return the latency in seconds
   */
  @Override
  public double getLatency() {
    return camResult.getTimestampSeconds();
  }

  /**
   * Get the target pose estimate relative to the robot.
   *
   * @return the target pose estimate relative to the robot
   */
  @Override
  public Optional<Pose3d> getRobotPose() {
    return Optional.empty();
  }

  /**
   * Get the target pose estimate relative to the robot.
   *
   * @return the target pose estimate relative to the robot
   */
  @Override
  public Optional<Pose3d> getRobotToTargetPose() {
    return Optional.empty();
  }

  /**
   * Get the confidence of the target detection
   *
   * @return the confidence of the target detection
   */
  @Override
  public double getConfidence() {
    return getRobotToTargetPose().map(it -> it.getTranslation().getNorm() / 10.0).orElse(Double.MAX_VALUE);
  }

  /**
   * Is the camera active?
   *
   * @return true if the camera is active and has a valid target
   */
  @Override
  public boolean isActive() {
    return hasValidTarget();
  }

  /**
   * Get the position of the target relative to the robot.
   *
   * @return the position of the target relative to the robot
   */
  @Override
  public Translation3d getPosition() {
    return getRobotPose().orElse(new Pose3d()).getTranslation();
  }

  /**
   * Get the rotation of the target relative to the robot.
   *
   * @return the rotation of the target relative to the robot
   */
  @Override
  public Rotation3d getRotation() {
    return getRobotPose().orElse(new Pose3d()).getRotation();
  }

  /**
   * Get the capture time of the camera in seconds.
   *
   * @return the capture time in seconds
   */
  public double getCaptureTime() {
    return getLatency();
  }
}
