// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public interface PoseProvider {

    public enum ProviderType {
        ALL,
        NONE,
        FIELD_BASED,
        ENVIRONMENT_BASED,
        RELATIVE
    }
    
    public static class PoseObservation {
        public double timestamp;
        public Vector<N3> stdDeviations;
        public Pose3d pose;
        public double ambiguity;
        public int tagCount;

        public PoseObservation(double timestamp, double ambiguity, int tagCount, Vector<N3> stdDeviations, Pose3d pose) {
            this.timestamp = timestamp; 
            this.ambiguity = ambiguity;
            this.tagCount = tagCount;
            this.stdDeviations = stdDeviations;
            this.pose = pose;
        }
    }

    /*
     * Returns the current pose of the robot.
     * 
     * @return The current pose of the robot.
     */
    public Optional<Pose3d> getRobotPose();

    /*
     * Returns the current observations of the robot.
     * 
     * @return The current observations of the robot.
     */
    public List<PoseObservation> getObservations();
    
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

    public ProviderType getType();
}
