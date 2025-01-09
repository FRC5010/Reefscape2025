// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.camera.GenericCamera;
import org.frc5010.common.sensors.camera.LimeLightCamera;
import org.frc5010.common.sensors.camera.PhotonVisionCamera;
import org.frc5010.common.sensors.camera.PhotonVisionFiducialTargetCamera;
import org.frc5010.common.sensors.camera.PhotonVisionPoseCamera;
import org.frc5010.common.sensors.camera.PhotonVisionVisualTargetCamera;
import org.frc5010.common.sensors.camera.QuestNav;
import org.frc5010.common.sensors.camera.SimulatedCamera;
import org.frc5010.common.sensors.camera.SimulatedFiducialTargetCamera;
import org.frc5010.common.sensors.camera.SimulatedVisualTargetCamera;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.subsystems.VisibleTargetSystem;
import org.frc5010.common.vision.AprilTags;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class CameraConfigurationJson {
  /** Limelight constant */
  public static String LIMELIGHT = "limelight";
  /** PhotonVision constant */
  public static String PHOTON_VISION = "photonvision";
  /** AprilTag constant */
  public static String APRIL_TAG = "apriltag";
  /** Target camera constant */
  public static String TARGET = "target";

  /** Name of the camera */
  public String name;
  /** Use of the camera */
  public String use;
  /** Type of the camera */
  public String type = "none";
  /** Optional strategy of the camera pose */
  public String strategy = "none";
  /** Column in SmartDashboard */
  public int column = 0;
  /** Camera X position from center in meters, in the robot's reference frame */
  public double x = 0;
  /** Camera Y position from center in meters, in the robot's reference frame */
  public double y = 0;
  /** Camera Z position from center in meters, in the robot's reference frame */
  public double z = 0;
  /** Camera roll angle in degrees, in the robot's reference frame */
  public double roll = 0;
  /** Camera pitch angle in degrees, in the robot's reference frame */
  public double pitch = 0;
  /** Camera yaw angle in degrees, in the robot's reference frame */
  public double yaw = 0;
  /** optional target height in meters */
  public double targetHeight = 0;
  /** optional target fiducial id */
  public int[] targetFiducialIds = new int[0];

  /**
   * Configures the camera system based on the provided robot.
   *
   * @param robot the GenericRobot instance to configure the camera for
   */
  public void configureCamera(GenericRobot robot) {
    GenericCamera camera = null;
    GenericDrivetrain drivetrain = (GenericDrivetrain) robot.getSubsystem("drivetrain");
    Transform3d robotToCamera = new Transform3d(new Translation3d(x, y, z),
        new Rotation3d(Degrees.of(roll), Degrees.of(pitch), Degrees.of(yaw)));
    AprilTagPoseSystem atSystem = (AprilTagPoseSystem) robot.getSubsystem(APRIL_TAG);
    if (atSystem == null) {
      atSystem = new AprilTagPoseSystem(AprilTags.aprilTagFieldLayout);
      robot.addSubsystem(APRIL_TAG, atSystem);
      drivetrain.getPoseEstimator().addAprilTagPoseSystem(atSystem);
    }

    if (RobotBase.isReal()) {
      switch (type) {
        case "limelight": {
          camera = new LimeLightCamera(name, column, robotToCamera);
          ((LimeLightCamera) camera)
              .setGyroSupplier(() -> (GenericGyro) robot.getDevice(ConfigConstants.GYRO));
          break;
        }
        case "photonvision": {
          if (!"none".equalsIgnoreCase(strategy)) {
            camera = new PhotonVisionPoseCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                PoseStrategy.valueOf(strategy),
                robotToCamera,
                robot.getPoseSupplier());
          } else if (targetFiducialIds.length > 0) {
            List<Integer> targetFiducialIdList = new ArrayList<>();
            for (int targetFiducialId : targetFiducialIds) {
              targetFiducialIdList.add(targetFiducialId);
            }
            camera = new PhotonVisionFiducialTargetCamera(
                name,
                column,
                AprilTags.aprilTagFieldLayout,
                robotToCamera,
                targetFiducialIdList);
          } else if (targetHeight > 0) {
            camera = new PhotonVisionVisualTargetCamera(name, column, robotToCamera);
          } else {
            camera = new PhotonVisionCamera(name, column, robotToCamera);
          }
          break;
        }
        default:
      }
    } else {
      if (!"none".equalsIgnoreCase(strategy)) {
        camera = new SimulatedCamera(
            name,
            column,
            AprilTags.aprilTagFieldLayout,
            PoseStrategy.valueOf(strategy),
            robotToCamera,
            robot.getSimulatedPoseSupplier());
      } else if (targetFiducialIds.length > 0) {
        List<Integer> targetFiducialIdList = new ArrayList<>();
        for (int targetFiducialId : targetFiducialIds) {
          targetFiducialIdList.add(targetFiducialId);
        }
        camera = new SimulatedFiducialTargetCamera(
            name,
            column,
            AprilTags.aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            robotToCamera,
            robot.getSimulatedPoseSupplier(),
            targetFiducialIdList);
      } else if (targetHeight > 0) {
        camera = new SimulatedVisualTargetCamera(
            name,
            column,
            AprilTags.aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            robotToCamera,
            robot.getSimulatedPoseSupplier());
      } else if (!"quest".equalsIgnoreCase(use)) {
        camera = new SimulatedCamera(
            name,
            column,
            AprilTags.aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            robotToCamera,
            robot.getSimulatedPoseSupplier());
      }
    }
    switch (use) {
      case "target":
        robot.addSubsystem(name, new VisibleTargetSystem(camera, targetHeight));
        break;
      case "apriltag": {
        if (drivetrain != null) {
          drivetrain.getPoseEstimator().registerPoseProvider(camera);
        }
        //atSystem.addCamera(camera);
        break;
      }
      case "quest": {
        QuestNav questNav = new QuestNav(robotToCamera);
        questNav.resetPose();
        if (drivetrain != null) {
          drivetrain.getPoseEstimator().registerPoseProvider(questNav);
        }
        break;
      }
    }
  }
}
