// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    visionLayout.addDouble("Observations", () -> input.poseObservations.length);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, cameraToRobot);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());

    List<PoseObservation> observations = new ArrayList<>();

    super.updateCameraInfo();
    SmartDashboard.putString("Primary Strategy " + name, poseEstimator.getPrimaryStrategy().name());
    Set<Short> tagIds = new HashSet<>();

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

        // Add tag IDs
        camResult.multitagResult.map(it -> tagIds.addAll(it.fiducialIDsUsed));

        SmartDashboard.putNumber("Total Distance To Tag " + name, totalTagDistance);
        SmartDashboard.putNumber("Photon Ambiguity " + name, camResult.getBestTarget().poseAmbiguity);
        SmartDashboard.putNumberArray("Photon Camera " + name + " POSE", new double[] {
            robotPose.getX(), robotPose.getY(), robotPose.getRotation().toRotation2d().getDegrees()
        });

        observations.add(
            new PoseObservation(
                camResult.getTimestampSeconds(), // Timestamp
                // 3D pose estimate
                robotPose,
                camResult.getBestTarget().poseAmbiguity,
                tagCount,
                averageDistance,
                PoseObservationType.PHOTONVISION,
                ProviderType.FIELD_BASED));
      }

      // Save pose observations to inputs object
      input.poseObservations = new PoseObservation[observations.size()];
      for (int i = 0; i < observations.size(); i++) {
        input.poseObservations[i] = observations.get(i);
      }
      // Save tag IDs to inputs objects
      input.tagIds = new int[tagIds.size()];
      int i = 0;
      for (int id : tagIds) {
        input.tagIds[i++] = id;
      }
    }
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 4.0) / observation.tagCount();
    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;

    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
    if (camResult.multitagResult.isEmpty()) {
      angularStdDev = 1.0;
    }
    //double rotStdDev = 0.3;

    // If really close, disregard angle measurement
    if (observation.averageTagDistance() < 0.3 || (observation.averageTagDistance() > 2 && RobotState.isEnabled())) {
      angularStdDev = 1000.0;
    }

    if ((observation.averageTagDistance() > 2.5 && RobotState.isEnabled() && observation.tagCount() < 2)) {
      linearStdDev = 100.0;
    }
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
