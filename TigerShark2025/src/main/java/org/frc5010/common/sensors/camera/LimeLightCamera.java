// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.LimelightHelpers;
import org.frc5010.common.vision.LimelightHelpers.PoseEstimate;

/** Limelight Camera */
public class LimeLightCamera extends GenericCamera {
  Optional<PoseEstimate> poseEstimate = Optional.empty();
  Optional<Pose3d> targetPose = Optional.empty();
  Supplier<GenericGyro> gyroSupplier;
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
    if (poseEstimate.isPresent() && poseEstimate.get().avgTagDist < 3) {
      getRobotPoseEstimateM1();
    }
    return poseEstimate;
  }

  @Override
  public int fiducialId() {
    return (int)LimelightHelpers.getFiducialID(name);
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

  /** Update the camera */
  @Override
  public void updateCameraInfo() {
    LimelightHelpers.getLatestResults(name);
    if (hasValidTarget()) {
      poseEstimate = megatagChooser.getAsBoolean() ? getRobotPoseEstimateM1() : getRobotPoseEstimateM2();
      targetPose = Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(name));
    }
  }

  /** Get the validity of the target information */
  @Override
  public boolean hasValidTarget() {
    return LimelightHelpers.getLimelightNTTableEntry(name, "tv").getDouble(0) == 1;
  }

  /**
   * Get the yaw of the target in degrees along the horizontal X axis of the
   * camera
   */
  @Override
  public double getTargetYaw() {
    return LimelightHelpers.getLimelightNTDouble(name, "tx");
  }

  /**
   * Get the pitch of the target in degrees along the vertical Y axis of the
   * camera
   */
  @Override
  public double getTargetPitch() {
    return LimelightHelpers.getLimelightNTDouble(name, "ty");
  }

  /** Get the area of the target in percentage of the image */
  @Override
  public double getTargetArea() {
    return LimelightHelpers.getLimelightNTDouble(name, "ta");
  }

  /**
   * Get the latency in seconds between the camera image and when the data was
   * received
   */
  @Override
  public double getLatency() {
    return poseEstimate.map(it -> it.timestampSeconds).orElse(0.0);
  }

  /** Get the current pose estimate of the robot */
  @Override
  public Optional<Pose3d> getRobotPose() {
    return Optional.ofNullable(poseEstimate.map(
        it -> null != it.pose ? new Pose3d(it.pose) : null).orElse(null));
  }

  /** Get the target pose estimate relative to the robot */
  @Override
  public Optional<Pose3d> getRobotToTargetPose() {
    return targetPose;
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

  public void setRobotToCameraOnLL() {
    LimelightHelpers.setCameraPose_RobotSpace(name, robotToCamera.getX(), -robotToCamera.getY(), robotToCamera.getZ(),
    robotToCamera.getRotation().getX(), robotToCamera.getRotation().getY(), robotToCamera.getRotation().getZ());
  }
  
  @Override
  public double getConfidence() {
    return 0.000000000000001;
  }

  @Override
  public boolean isActive() {
    return hasValidTarget();
  }

  @Override
  public Translation3d getPosition() {
    return getRobotPose().orElse(new Pose3d()).getTranslation();
  }

  @Override
  public Rotation3d getRotation() {
    return getRobotPose().orElse(new Pose3d()).getRotation();
  }

  public double getCaptureTime() {
    return getLatency();
  }

  public void resetPose(Pose3d init) {}
}
