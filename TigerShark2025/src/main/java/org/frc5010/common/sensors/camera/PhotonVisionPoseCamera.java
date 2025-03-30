// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.frc5010.common.vision.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** A camera using the PhotonVision library. */
public class PhotonVisionPoseCamera extends PhotonVisionCamera {
  /** The pose estimator */
  protected PhotonPoseEstimator poseEstimator;
  /** The pose strategy */
  protected PoseStrategy strategy;
  /** The pose supplier */
  protected Supplier<Pose2d> poseSupplier;
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();

  protected List<PoseObservation> observations = new ArrayList<>();
  protected int tagCount = 0;
  protected double averageDistance = 0;

  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param fieldLayout   - the field layout
   * @param strategy      - the pose strategy
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier  - the pose supplier
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
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    
  }

  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds) {
    super(name, colIndex, cameraToRobot);
    this.strategy = strategy;
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;
    this.fiducialIds = fiducialIds;
    visionLayout.addDouble("Observations", () -> observations.size());
    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, cameraToRobot);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());
  
    observations.clear();

    super.updateCameraInfo();
    SmartDashboard.putString("Primary Strategy "+name, poseEstimator.getPrimaryStrategy().name());
  
    for (PhotonPipelineResult camResult : camResults) {
      Optional<EstimatedRobotPose> estimate = poseEstimator.update(camResult);
      if (estimate.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose3d robotPose = estimatedRobotPose.estimatedPose;
        
        double totalTagDistance = 0.0;
        for (var target : camResult.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        // Compute the average tag distance
        int tagCount = estimatedRobotPose.targetsUsed.size();
        double averageDistance = 0.0;
        if (camResult.targets.size() > 0) {
          averageDistance = totalTagDistance / camResult.targets.size();
        }

        double stdDevFactor = Math.pow(averageDistance, 4.0) / tagCount;
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;


        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
        if (camResult.multitagResult.isEmpty()) {
          angularStdDev = 1.0;
        }


        double rotStdDev = 0.3;

        // If really close, disregard angle measurement
        if (totalTagDistance < 0.3) {
          angularStdDev = 1000.0;
        }
        SmartDashboard.putNumber("Total Distance To Tag "+name, totalTagDistance);
        SmartDashboard.putNumber("Photon Ambiguity "+name,  camResult.getBestTarget().poseAmbiguity);
        SmartDashboard.putNumberArray("Photon Camera "+name+" POSE", new double[] {
                  robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d().getDegrees()
          });

        if (RobotState.isEnabled() && robotPose.getTranslation().toTranslation2d().getDistance(poseSupplier.get().getTranslation()) > 2.0) {
          continue;
        }

        observations.add(
            new PoseObservation(
                camResult.getTimestampSeconds(), // Timestamp
                camResult.getBestTarget().poseAmbiguity,
                tagCount,
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                robotPose // 3D pose estimate
            ));
      }
      // if (camResult.hasTargets()) {
        
      //   if (camResult.getMultiTagResult().isPresent()) {
      //     var multitagResult = camResult.multitagResult.get();
          

      //     // Calculate robot pose
      //     Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      //     Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      //     Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      //     // Calculate average tag distance
      //     double totalTagDistance = 0.0;
      //     for (var target : camResult.targets) {
      //       totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      //     }
      //     // Compute the average tag distance
      //     int tagCount = multitagResult.fiducialIDsUsed.size();
      //     double averageDistance = 0.0;
      //     if (camResult.targets.size() > 0) {
      //       averageDistance = totalTagDistance / camResult.targets.size();
      //     }

      //     double stdDevFactor = Math.pow(averageDistance, 2.0) / tagCount;
      //     double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
      //     double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
      //     SmartDashboard.putNumberArray("Photon Camera "+name+" POSE", new double[] {
      //               robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d().getDegrees()
      //       });
      //     observations.add(
      //         new PoseObservation(
      //             camResult.getTimestampSeconds(), // Timestamp
      //             multitagResult.estimatedPose.ambiguity,
      //             tagCount,
      //             VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
      //             robotPose // 3D pose estimate
      //         ));
      //   }
      //   if (fiducialIds.size() > 0) {
      //     target = camResult.getTargets().stream()
      //         .filter(it -> fiducialIds.contains(it.getFiducialId()))
      //         .findFirst();
      //   } else {
      //     target = Optional.ofNullable(camResult.getBestTarget());
      //   }
      // }
    }
  }

  @Override
  public void update() {
    updateCameraInfo();
  }

  @Override
  public List<PoseObservation> getObservations() {
    return observations;
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
