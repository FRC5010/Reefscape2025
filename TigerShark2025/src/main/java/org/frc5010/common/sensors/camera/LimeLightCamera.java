// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.LimelightHelpers;
import org.frc5010.common.vision.LimelightHelpers.PoseEstimate;
import org.frc5010.common.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Limelight Camera */
public class LimeLightCamera extends GenericCamera {
  Optional<Pose3d> targetPose = Optional.empty();
  Supplier<GenericGyro> gyroSupplier;
  Supplier<Rotation2d> angleResetSupplier;
  BooleanSupplier megatagChooser;

  /**
   * Constructor with megatag chooser for AprilTags
   *
   * @param name           the name of the camera, 'limelight-' will be prepended
   *                       to the name
   * @param colIndex       the column index of the display tab
   * @param megatagChooser boolean supplier to choose between M1 and M2 for pose
   *                       estimation
   */
  public LimeLightCamera(String name, int colIndex, BooleanSupplier megatagChooser) {
    super("limelight-" + name, colIndex, new Transform3d());
    this.megatagChooser = megatagChooser;
  }

  /**
   * Constructor without megatag chooser, M1 is always chosen
   *
   * @param name     the name of the camera, 'limelight-' will be prepended to the
   *                 name
   * @param colIndex the column index of the display tab
   */
  public LimeLightCamera(String name, int colIndex) {
    super("limelight-" + name, colIndex, new Transform3d());
    this.megatagChooser = () -> true;
  }

  /**
   * Constructor with megatag chooser for AprilTags or targets
   *
   * @param name           the name of the camera, 'limelight-' will be prepended
   *                       to the name
   * @param colIndex       the column index of the display tab
   * @param cameraToRobot  the camera position relative to the robot's center
   * @param megatagChooser boolean supplier to choose between M1 and M2 for pose
   *                       estimation
   */
  public LimeLightCamera(
      String name, int colIndex, Transform3d cameraToRobot, BooleanSupplier megatagChooser) {
    super("limelight-" + name, colIndex, cameraToRobot);
    this.megatagChooser = megatagChooser;
    setRobotToCameraOnLL();
  }

  /**
   * Constructor for targeting cameras without megatag chooser
   *
   * @param name          the name of the camera, 'limelight-' will be prepended
   *                      to the name
   * @param colIndex      the column index of the display tab
   * @param cameraToRobot the camera position relative to the robot's center
   */
  public LimeLightCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super("limelight-" + name, colIndex, cameraToRobot);
    this.megatagChooser = () -> true;
    setRobotToCameraOnLL();
  }

  /**
   * Get the robot's estimated pose using the Megatag 1 algorithm
   *
   * @return the robot's estimated pose
   */
  protected Optional<PoseEstimate> getRobotPoseEstimateM1() {
    Optional<PoseEstimate> poseEstimate = validatePoseEstimate(
        Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue(name)));

    if (poseEstimate.isPresent() && null != poseEstimate.get().pose && null != gyroSupplier) {
      SmartDashboard.putNumber("MT1 Angle", poseEstimate.get().pose.getRotation().getDegrees());
      gyroSupplier.get().setAngle(poseEstimate.get().pose.getRotation().getDegrees());
    }

    return poseEstimate;
  }

  /**
   * Get the robot's estimated pose using the Megatag 2 algorithm
   *
   * @return the robot's estimated pose
   */
  protected Optional<PoseEstimate> getRobotPoseEstimateM2() {
    Optional<PoseEstimate> poseEstimate = validatePoseEstimate(
        Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)));
    return poseEstimate;
  }
  
  /**
   * Validate the pose estimate
   *
   * @param poseEstimate the pose estimate
   * @return an optional containing the pose estimate, if valid
   */
  private Optional<PoseEstimate> validatePoseEstimate(Optional<PoseEstimate> poseEstimate) {
    if (poseEstimate.isPresent()) {
      Pose2d pose = poseEstimate.get().pose;

      if (poseEstimate.get().tagCount == 0) {
        pose = null;
      }

      if (null != gyroSupplier && Math.abs(gyroSupplier.get().getRate()) > 180) {
        pose = null;
      }

      poseEstimate.get().pose = pose;
    }
    return poseEstimate;
  }

  /**
   * Set the Megatag pose estimation chooser
   *
   * @param chooser the boolean supplier that selects M1 when true, M2 when false
   */
  public void setPoseEstimationChooser(BooleanSupplier chooser) {
    megatagChooser = chooser;
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  /** Update the camera */
  @Override
  public void updateCameraInfo() {
    if (null != gyroSupplier.get()) {
      GenericGyro gyro = gyroSupplier.get();
      LimelightHelpers.SetRobotOrientation(name, gyro.getAngle(), gyro.getRate(), 0.0, 0.0, 0.0, 0.0);
    } else if (null != angleResetSupplier.get()) {
      LimelightHelpers.SetRobotOrientation(name, angleResetSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    LimelightHelpers.getLatestResults(name);
    List<PoseObservation> observations = new ArrayList<>();
    if (hasValidTarget()) {
      Optional<PoseEstimate> poseEstimate = megatagChooser.getAsBoolean() ? getRobotPoseEstimateM1() : getRobotPoseEstimateM2();
      if (poseEstimate.isPresent()) {
        Pose2d currPose = poseEstimate.get().pose;
        if (null != currPose) {
          SmartDashboard.putNumberArray("Limelight POSE", new double[] {
              currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()
          });
        }
      }
      targetPose = Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(name));
      input.latestTargetPose = targetPose.orElse(new Pose3d());
      input.hasTarget = targetPose.isPresent();

      observations.add(
          new PoseObservation(
              poseEstimate.map(it -> it.timestampSeconds).orElse(0.0),
              // 3D pose estimate
              poseEstimate.map(it -> new Pose3d(it.pose)).orElse(null),
              determineConfidence(poseEstimate.get()),
              poseEstimate.map(it -> it.tagCount).orElse(0),
              poseEstimate.map(it -> it.avgTagDist).orElse(0.0),
              PoseObservationType.PHOTONVISION,
              ProviderType.FIELD_BASED));

      // Save pose observations to inputs object
      if (observations.size() != input.poseObservations.length) {
        input.poseObservations = new PoseObservation[observations.size()];
      }
      for (int i = 0; i < observations.size(); i++) {
        input.poseObservations[i] = observations.get(i);
      }

      // Save tag IDs to inputs objects
      input.tagIds = Arrays.stream(LimelightHelpers.getRawFiducials(name))
                            .mapToInt(fiducial -> fiducial.id)
                            .distinct()
                            .toArray();
    }
  }

  /** Get the area of the target in percentage of the image */
  @Override
  public double getTargetArea() {
    return LimelightHelpers.getLimelightNTDouble(name, "ta");
  }

  /**
   * Set the supplier for the gyroscope
   *
   * @param gyroSupplier the supplier for the gyroscope
   * @return this camera
   */
  public LimeLightCamera setGyroSupplier(Supplier<GenericGyro> gyroSupplier) {
    this.gyroSupplier = gyroSupplier;
    return this;
  }

  public LimeLightCamera setAngleSupplier(Supplier<Rotation2d> rotationSupplier) {
    this.angleResetSupplier = rotationSupplier;
    return this;
  }

  public void setRobotToCameraOnLL() {
    LimelightHelpers.setCameraPose_RobotSpace(name, robotToCamera.getX(), -robotToCamera.getY(), robotToCamera.getZ(),
        robotToCamera.getRotation().getX(), robotToCamera.getRotation().getY(), robotToCamera.getRotation().getZ());
  }

  public double determineConfidence( PoseEstimate estimate) {
    Stream<RawFiducial> fiducialStream = Arrays.stream(LimelightHelpers.getRawFiducials(name))
        .sorted((raw1, raw2) -> raw1.ambiguity == raw2.ambiguity ? 0 : (raw1.ambiguity > raw2.ambiguity ? 1 : -1));
    double min_ambiguity = fiducialStream.findFirst().map(fiducial -> fiducial.ambiguity).orElse(100.0);
    double confidence = estimate.avgTagDist > 2 ? 1.0 : min_ambiguity * Math.max(estimate.avgTagDist / 2, 0.5);
    SmartDashboard.putNumber("Limelight Confidence", confidence);
    return confidence;
  }

  @Override
  public ProviderType getType() {
    return ProviderType.FIELD_BASED;
  }
}
