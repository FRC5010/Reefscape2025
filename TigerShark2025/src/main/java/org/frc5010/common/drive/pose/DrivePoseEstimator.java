// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.camera.QuestNav;
import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.vision.AprilTags;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A class to handle estimating the pose of the robot */
public class DrivePoseEstimator extends GenericSubsystem {
  /** The pose tracker */
  protected GenericPose poseTracker;
  /** The vision system used to get the robot pose */
  protected AprilTagPoseSystem vision;
  /** The field2d object for displaying the pose */
  private final Field2d field2d;
  /** The list of AprilTag poses */
  private List<Pose2d> tagPoses = new ArrayList<>();
  /** Whether to disable the vision update */
  private boolean disableVisionUpdate = false;
    /** Whether to disable the vision update command */
    private boolean disableVisionUpdateCommand = false;
  /** List of PoseProviders */
  private List<PoseProvider> poseProviders = new ArrayList<>();

  /**
   * Build a DrivePoseEstimator
   *
   * @param poseTracker the pose tracker
   * @param vision the vision system
   */
  public DrivePoseEstimator(GenericPose poseTracker, AprilTagPoseSystem vision) {
    this.poseTracker = poseTracker;
    this.vision = vision;
    field2d = poseTracker.getField();
    //poseProviders.addAll(vision.getPoseProviders());
    disableVisionUpdate = true;

    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(11, 0);
    tab.addDoubleArray("Robot Pose3d", () -> getCurrentPose3dArray()).withPosition(11, 1);

    tab.addNumber("Pose Degrees", () -> (getCurrentPose().getRotation().getDegrees()))
        .withPosition(11, 2);
    tab.add("Pose Field", field2d).withPosition(0, 0).withSize(11, 5);

    for (AprilTag at : vision.getFieldLayout().getTags()) {
      if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
        field2d.getObject("Field Tag " + at.ID).setPose(at.pose.toPose2d());
        AprilTags.poseToID.put(at.pose.toPose2d(), at.ID);
        tagPoses.add(at.pose.toPose2d());
      }
    }
  }
    /**
   * Build a DrivePoseEstimator
   *
   * @param poseTracker the pose tracker
   * @param vision the vision system
   */
  public DrivePoseEstimator(GenericPose poseTracker) {
    this.poseTracker = poseTracker;
    field2d = poseTracker.getField();
    disableVisionUpdate = true;

    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(11, 0);
    tab.addDoubleArray("Robot Pose3d", () -> getCurrentPose3dArray()).withPosition(11, 1);

    tab.addNumber("Pose Degrees", () -> (getCurrentPose().getRotation().getDegrees()))
        .withPosition(11, 2);
    tab.add("Pose Field", field2d).withPosition(0, 0).withSize(11, 5);

    for (AprilTag at : AprilTags.aprilTagFieldLayout.getTags()) {
      if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
        field2d.getObject("Field Tag " + at.ID).setPose(at.pose.toPose2d());
        AprilTags.poseToID.put(at.pose.toPose2d(), at.ID);
        tagPoses.add(at.pose.toPose2d());
      }
    }
  }

  /**
   * Register a PoseProvider to the list of pose providers.
   *
   * @param provider the PoseProvider to be registered
   */
  public void registerPoseProvider(PoseProvider provider) {
    poseProviders.add(provider);
  }
  
  /**
   * Set whether to disable the vision update
   *
   * @param disable whether to disable the vision update
   */
  public void setDisableVisionUpdate(boolean disable) {
    disableVisionUpdate = disable;
  }

  /**
   * Get the formatted pose as a String
   *
   * @return the formatted pose
   */
  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
  }

  /**
   * Get the current pose on the 2D plane
   *
   * @return the current pose
   */
  public Pose2d getCurrentPose() {
    //return poseProviders.get(0).getRobotPose().get().toPose2d();
    return poseTracker.getCurrentPose();
  }

  /**
   * Get the current pose in 3d space
   *
   * @return the current pose with z = 0
   */
  public Pose3d getCurrentPose3d() {
    return new Pose3d(poseTracker.getCurrentPose());
  }

  /**
   * Get the current pose in 3d space as an array
   *
   * @return the current pose
   */
  public double[] getCurrentPose3dArray() {
    Pose3d pose = getCurrentPose3d();
    Quaternion rotation = pose.getRotation().getQuaternion();
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      rotation.getW(),
      rotation.getX(),
      rotation.getY(),
      rotation.getZ()
    };
  }

  /**
   * Update the pose estimator.
   *
   * <p>This method updates the local measurements of the pose tracker and checks if vision update
   * is disabled. If vision update is not disabled, it retrieves the list of camera names from the
   * vision object and iterates over each camera. For each camera, it sets a boolean value on the
   * SmartDashboard to indicate that the camera is being processed. It then retrieves the robot pose
   * and distance to target from the vision object for the current camera. If a robot pose is
   * present, it retrieves the image capture time and updates the vision measurements of the pose
   * tracker. Finally, it updates a boolean value on the SmartDashboard to indicate if any vision
   * updates were made. After updating the vision measurements, it sets the robot pose in the
   * field2d object using the current pose.
   */
  public void update() {
    poseTracker.updateLocalMeasurements();
    if (!disableVisionUpdate) {
      List<String> cameras = vision.getCameraNames();
      boolean visionUpdated = false;
      for (String camera : cameras) {
        SmartDashboard.putBoolean(camera, false);
        Optional<Pose3d> robotPose = vision.getRobotPose3d(camera);
        double poseDistance = vision.getDistanceToTarget(camera);
        if (robotPose.isPresent()) {
          double imageCaptureTime = vision.getLatency(camera);
          visionUpdated = true;
          SmartDashboard.putBoolean(camera, true);
          if (4 > poseDistance) {
            poseTracker.updateVisionMeasurements(
              robotPose.get().toPose2d(), imageCaptureTime, vision.getStdVector(poseDistance));
          }
        }
      }
      SmartDashboard.putBoolean("April Tag Pose Updating", visionUpdated);
    }
    poseTracker.updateRobotPoseOnField(getCurrentPose());
  }

  @Override
  public void periodic() {
    poseProviders.forEach(it -> it.update());
    updatePoseFromProviders();
  }

  private void resetProviderPoses(Pose2d pose) {
    for (PoseProvider provider : poseProviders) {
      provider.resetPose(new Pose3d(pose));
    }
  }

  /**
   * Creates a command that updates the pose estimator using the pose providers.
   *
   * <p>This command will update the local measurements of the pose tracker and check if vision
   * update is disabled. If vision update is not disabled, it will iterate over each pose provider
   * and update the vision measurements of the pose tracker if a robot pose is present. Finally, it
   * will update a boolean value on the SmartDashboard to indicate if any vision updates were made.
   *
   * @return the command that updates the pose estimator
   */
  protected void updatePoseFromProviders() {
      poseTracker.updateLocalMeasurements();
      boolean visionUpdated = false;
      if (!disableVisionUpdateCommand) {
        for (PoseProvider provider: poseProviders) {
          if (provider.isActive()) {
            Optional<Pose3d> robotPose = provider.getRobotPose();
            if (robotPose.isPresent()) {
              double confidence = provider.getConfidence();
              visionUpdated |= provider.fiducialId() != 0;
              poseTracker.updateVisionMeasurements(
                robotPose.get().toPose2d(), provider.getCaptureTime(), vision.getStdConfidenceVector(confidence));
            }
          }
        }
      }  
      SmartDashboard.putBoolean("April Tag Pose Updating", visionUpdated);
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating to the software
   * when you have manually moved your robot in a particular position on the field (EX: when you
   * place it on the field at the start of the match).
   *
   * @param pose the pose to reset to
   */
  public void resetToPose(Pose2d pose) {
    poseTracker.resetToPose(pose);
    resetProviderPoses(pose);
  }

  /**
   * Get the current gyro rotation as a {@link Rotation2d}.
   *
   * @return the current gyro rotation
   */
  public Rotation2d getGyroRotation2d() {
    // System.out.println(poseTracker.getGyroRotation2d());
    return poseTracker.getGyroRotation2d();
  }

  /**
   * Get the closest tag to the robot
   *
   * @return the ID of the closest tag
   */
  public int getClosestTagToRobot() {
    return AprilTags.poseToID.get(getCurrentPose().nearest(tagPoses));
  }

  /**
   * Set the target pose on the field
   *
   * @param targetPose the target pose
   * @param targetName the name of the target
   */
  public void setTargetPoseOnField(Pose2d targetPose, String targetName) {
    field2d.getObject(targetName).setPose(targetPose);
  }

  /**
   * Get the pose from the closest tag as a {@link Pose3d}
   *
   * @return the pose of the closest tag
   */
  public Pose3d getPoseFromClosestTag() {
    Pose3d targetPose =
        vision.getFieldLayout().getTagPose(getClosestTagToRobot()).orElse(getCurrentPose3d());
    field2d.getObject("Closest Tag").setPose(targetPose.toPose2d());
    return targetPose;
  }
  public void addAprilTagPoseSystem(AprilTagPoseSystem atSystem) {
    vision = atSystem;
  }
}
