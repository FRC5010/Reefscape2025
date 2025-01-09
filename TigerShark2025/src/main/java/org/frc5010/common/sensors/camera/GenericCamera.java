// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.frc5010.common.drive.pose.PoseProvider;
import org.frc5010.common.vision.VisionConstants;

/** A generic camera interface */
public abstract class GenericCamera implements PoseProvider {
  /** The list of updaters that will be called every time the camera is updated */
  protected List<Runnable> updaters = new ArrayList<>();
  /** The robot-to-camera transform */
  protected Transform3d robotToCamera = new Transform3d();
  /** The display tab to use */
  protected ShuffleboardTab visionTab;
  /** The display layout to use */
  protected ShuffleboardLayout visionLayout;
  /** Whether or not to update the values */
  protected boolean updateValues = false;
  /** The index of the column in the dashboard */
  protected int colIndex;
  /** The name of the camera */
  protected String name;

  /**
   * Create a new camera
   *
   * @param name the name of the camera
   * @param colIndex the index of the column in the dashboard
   * @param robotToCamera the robot-to-camera transform
   */
  public GenericCamera(String name, int colIndex, Transform3d robotToCamera) {
    this.colIndex = colIndex;
    this.name = name;
    this.robotToCamera = robotToCamera;
    visionTab = Shuffleboard.getTab(VisionConstants.SBTabVisionDisplay);
    visionLayout =
        visionTab
            .getLayout("Camera " + this.name, BuiltInLayouts.kGrid)
            .withSize(2, 5)
            .withPosition(this.colIndex, 0);
    visionLayout.addBoolean("Has Target", this::hasValidTarget);
    visionLayout.addDouble("Target Yaw", this::getTargetYaw);
    visionLayout.addDouble("Target Pitch", this::getTargetPitch);
    visionLayout.addDouble("Target Area", this::getTargetArea);
    visionLayout.addDouble("Latency", this::getLatency);
    visionLayout.addDouble(
        "Pose X", () -> getRobotPose().orElse(new Pose3d(-1, -1, 0, new Rotation3d())).getX());
    visionLayout.addDouble(
        "Pose Y", () -> getRobotPose().orElse(new Pose3d(-1, -1, 0, new Rotation3d())).getY());
  }

  /**
   * Updates the state of the object by running all registered updaters.
   *
   * <p>This method iterates over the list of updaters and calls the `run()` method on each one.
   * This allows for the object to be updated based on the current state of the program.
   *
   * @throws NullPointerException if any of the updaters are null
   */
  public void update() {
    updateCameraInfo();
    for (Runnable updater : updaters) {
      updater.run();
    }
  }

  /** Updates the vision information from the camera */
  public void updateCameraInfo() {}

  /**
   * Registers a Runnable updater to the list of updaters.
   *
   * @param updater the Runnable updater to be added
   */
  public void registerUpdater(Runnable updater) {
    updaters.add(updater);
  }

  /**
   * Returns the name of the object.
   *
   * @return the name of the object
   */
  public String name() {
    return name;
  }

  /**
   * Returns the transformation matrix from the robot's coordinate system to the camera's coordinate
   * system.
   *
   * @return the transformation matrix from the robot's coordinate system to the camera's coordinate
   *     system
   */
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  /**
   * Removes the specified updater from the list of updaters.
   *
   * @param updater the updater to be removed
   */
  public void unregisterUpdater(Runnable updater) {
    updaters.remove(updater);
  }

  /**
   * Returns whether or not the camera has a valid target.
   *
   * @return whether or not the camera has a valid target
   */
  public abstract boolean hasValidTarget();

  /**
   * Returns the yaw of the target in degrees along the horizontal X axis of the camera.
   *
   * @return the yaw of the target in degrees along the horizontal X axis of the camera
   */
  public abstract double getTargetYaw();

  /**
   * Returns the pitch of the target in degrees along the vertical Y axis of the camera.
   *
   * @return the pitch of the target in degrees along the vertical Y axis of the camera
   */
  public abstract double getTargetPitch();

  /**
   * Returns the area of the target.
   *
   * @return the area of the target
   */
  public abstract double getTargetArea();

  /**
   * Returns the latency of the camera image.
   *
   * @return the latency of the camera image
   */
  public abstract double getLatency();

  /**
   * Returns the current pose estimate of the robot.
   *
   * @return the current pose estimate of the robot
   */
  public abstract Optional<Pose3d> getRobotPose();

  /**
   * Returns the target pose estimate relative to the robot.
   *
   * @return the target pose estimate relative to the robot
   */
  public abstract Optional<Pose3d> getRobotToTargetPose();
}
