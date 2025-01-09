// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    super(name, colIndex, fieldLayout, strategy, cameraToRobot, poseSupplier);
    if (!tagsLoaded) {
      visionSim.addAprilTags(fieldLayout);
      tagsLoaded = true;
    }

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
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

  /** Update the simulated camera */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    Pose2d p = poseSupplier.get();
    visionSim.update(p);
    visionSim.resetRobotPose(p);
  }
}
