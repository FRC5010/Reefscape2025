// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class YAGSLSwervePose extends GenericPose {
  private YAGSLSwerveDrivetrain drivetrain;

  public YAGSLSwervePose(YAGSLSwerveDrivetrain drivetrain) {
    super(null);
    this.drivetrain = drivetrain;
    field2d = drivetrain.getField2d();
    visionConsumer = drivetrain::updateVisionMeasurements;
  }

  @Override
  public void resetEncoders() {
    drivetrain.resetEncoders();
  }

  @Override
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Matrix<N3, N1> stdVector) {
    drivetrain.updateVisionMeasurements(robotPose, imageCaptureTime, stdVector);
  }

  @Override
  public void updateRobotPoseOnField(Pose2d pose) {
    field2d.setRobotPose(pose);
  }

  @Override
  public void updateLocalMeasurements() {}

  @Override
  public Pose2d getCurrentPose() {
    return drivetrain.getPose();
  }

  @Override
  public void resetToPose(Pose2d pose) {
    drivetrain.resetOdometry(pose);
  }

  @Override
  public Rotation2d getGyroRotation2d() {
    return drivetrain.getHeading();
  }
}
