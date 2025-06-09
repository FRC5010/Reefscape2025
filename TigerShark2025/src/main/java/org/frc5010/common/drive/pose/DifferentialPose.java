// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.gyro.GenericGyro;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/** A pose estimator for differential drive */
public class DifferentialPose extends GenericPose {
  private GenericEncoder leftEncoder;
  private GenericEncoder rightEncoder;
  private final DifferentialDrivePoseEstimator poseEstimator;

  /**
   * Create a pose estimator for differential drive
   *
   * @param kinematics differential drive kinematics
   * @param gyro gyroscope
   * @param leftEncoder left encoder
   * @param rightEncoder right encoder
   */
  public DifferentialPose(
      DifferentialDriveKinematics kinematics,
      GenericGyro gyro,
      GenericEncoder leftEncoder,
      GenericEncoder rightEncoder) {
    super(gyro);
    this.leftEncoder = leftEncoder;
    this.rightEncoder = rightEncoder;
    poseEstimator =
        new DifferentialDrivePoseEstimator(
            kinematics,
            getGyroRotation2d(),
            leftEncoder.getPosition(),
            rightEncoder.getPosition(),
            new Pose2d(),
            localMeasurementStdDevs,
            visionMeasurementStdDevs);
  }

  @Override
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Matrix<N3, N1> stdVector) {
    // poseEstimator.resetPosition(robotPose, robotPose.getRotation());
    // m_poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
  }

  public void updateLocalMeasurements() {
    double leftDist = leftEncoder.getPosition();
    double rightDist = rightEncoder.getPosition();
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), getGyroRotation2d(), leftDist, rightDist);
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void resetToPose(Pose2d pose) {
    poseEstimator.resetPosition(
        getGyroRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }
}
