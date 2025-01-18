// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public interface PoseProvider {
    
    /*
     * Returns the current pose of the robot.
     * 
     * @return The current pose of the robot.
     */
    public Optional<Pose3d> getRobotPose();

    /*
     * Returns the confidence of the current pose measurement. Used in order to merge multiple pose measurements. Lower values are better.
     *
     * @return The confidence of the current pose measurement.
     */
    public double getConfidence();


    /*
     * Returns whether the pose provider is currently active. A camera could be inactive if it is not currently tracking a target, for example.
     * 
     * @return Whether the pose provider is currently active.
     */ 
    public boolean isActive();

    /*
     * Returns the position of the pose
     * 
     * @return The position of the pose
     */
    public Translation3d getPosition();

    /*
     * Returns the rotation of the pose
     * 
     * @return The rotation of the pose
     */
    public Rotation3d getRotation();

    public double getCaptureTime();

    public void update();

    public void resetPose(Pose3d initPose);

    public int fiducialId();
}
