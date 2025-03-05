// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** A simulated camera using the PhotonVision library. */
public class SimulatedCamera extends PhotonVisionPoseCamera {
  /** The vision simulation system */
  public static VisionSystemSim visionSim = new VisionSystemSim("main");
  /** Whether the tags have been loaded */
  static boolean tagsLoaded = false;

  /** The simulated camera properties */
  protected SimCameraProperties cameraProp = new SimCameraProperties();
  /** The simulated camera */
  protected PhotonCameraSim cameraSim;
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
   * @param width         - the camera width
   * @param height        - the camera height
   * @param fov           - the camera field of view
   */
  public SimulatedCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      int width, int height, double fov) {
    super(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier);
    if (!tagsLoaded) {
      visionSim.addAprilTags(fieldLayout);
      tagsLoaded = true;
    }

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(width, height, Rotation2d.fromDegrees(fov));
    // Approximate detection noise with average and standard deviation error in
    // pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    cameraProp.setFPS(40);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addCamera(cameraSim, cameraToRobot);

    // Enable the raw and processed streams. These are enabled by default.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(true);
  }

  /**
   * Constructor
   *
   * @param name          - the name of the camera
   * @param colIndex      - the column index for the dashboard
   * @param fieldLayout   - the field layout
   * @param strategy      - the pose strategy
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier  - the pose supplier
   * @param fiducialIds   - the list of fiducial IDs
   * @param width         - the width of the camera
   * @param height        - the height of the camera
   * @param fov           - the field of view
   */
  public SimulatedCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds,
      int width, int height, double fov) {
    this(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier);
    this.fiducialIds = fiducialIds;
    visionLayout.addDouble("Target ID", () -> target.map(it -> it.getFiducialId()).orElse(-1));
  }

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
  public SimulatedCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier) {
    this(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier, 640, 480, 70.0);
  }

  /** Update the simulated camera */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      if (fiducialIds.size() > 0) {
        target = camResult.getTargets().stream()
            .filter(it -> fiducialIds.contains(it.getFiducialId()))
            .findFirst();
      } else {
        target = Optional.ofNullable(camResult.getBestTarget());
      }
    }
    Pose2d p = poseSupplier.get();
    visionSim.update(p);
    visionSim.resetRobotPose(p);
  }

  @Override
  public boolean hasValidTarget() {
    return fiducialIds.size() > 0 ? target.isPresent() : super.hasValidTarget();
  }
}
