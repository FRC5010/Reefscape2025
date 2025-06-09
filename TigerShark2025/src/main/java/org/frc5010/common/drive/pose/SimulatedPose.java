// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import org.frc5010.common.sensors.gyro.GenericGyro;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class SimulatedPose extends GenericPose {
  Pose2d pose = new Pose2d();

  public SimulatedPose(GenericGyro gyro) {
    super(gyro);
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Matrix<N3, N1> stdVector) {
    pose = robotPose;
  }

  @Override
  public void updateLocalMeasurements() {}

  @Override
  public Pose2d getCurrentPose() {
    return pose;
  }

  @Override
  public void resetToPose(Pose2d pose) {
    this.pose = pose;
    this.gyro.setAngle(pose.getRotation().getDegrees());
  }
}
