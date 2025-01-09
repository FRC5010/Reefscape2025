// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class PoseManager extends GenericSubsystem {
    private Map<String, PoseProvider> poseProviders = new HashMap<>();
    private Pose3d currentPose;
    private double currentPoseConfidence = 1.0;

    public PoseManager() {
        
    }

    public void addPoseProvider(String name, PoseProvider poseProvider) {
        poseProviders.put(name, poseProvider);
    }

    public PoseProvider getPoseProvider(String name) {
        return poseProviders.get(name);
    }

    public void updatePose() {
        Translation3d position = currentPose.getTranslation().times(1 / currentPoseConfidence);
        Rotation3d rotation = currentPose.getRotation().times(1 / currentPoseConfidence);
        double confidence = 1 / currentPoseConfidence;

        for (PoseProvider poseProvider : poseProviders.values()) {
            double inverse = 1 / poseProvider.getConfidence();
            if (poseProvider.isActive()) {
                position.plus(poseProvider.getPosition().times(inverse));
                rotation.plus(poseProvider.getRotation().times(inverse));
                confidence += inverse;
            }
        }
        currentPose = new Pose3d(position.div(1 / confidence), rotation.div(1 / confidence));
    }

    public void resetPose(Pose3d pose) {
        currentPose = pose;
    }

    public Pose3d getCurrentPose() {
        return currentPose;
    }

    @Override
    public void periodic() {
        updatePose();
    }



}
