// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5010.common.sensors.gyro.GenericGyro;

/** Base class for all pose estimators */
public abstract class GenericPose {
  /** The 2D field */
  protected Field2d field2d;
  /** State standard deviations */
  protected Matrix<N5, N1> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
  /** Odometry measurement standard deviations */
  protected Matrix<N3, N1> localMeasurementStdDevs =
      VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  /** Vision measurement standard deviations */
  protected Matrix<N3, N1> visionMeasurementStdDevs =
      VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

  /** The gyro */
  protected GenericGyro gyro;

  /**
   * Constructor for GenericPose
   *
   * @param gyro gyro
   */
  public GenericPose(GenericGyro gyro) {
    this.gyro = gyro;
  }

  /** Reset the encoders. */
  public abstract void resetEncoders();

  /**
   * Get the Display field.
   *
   * @return the Display field
   */
  public Field2d getField() {
    if (null == field2d) {
      field2d = new Field2d();
    }
    return field2d;
  }

  public void updateRobotPoseOnField(Pose2d pose) {
    field2d.setRobotPose(pose);
  }
  /**
   * Get the current acceleration in the X direction.
   *
   * @return the current acceleration in the X direction
   */
  public double getAccelX() {
    return 0;
  }

  /**
   * Get the current acceleration in the Y direction.
   *
   * @return the current acceleration in the Y direction
   */
  public double getAccelY() {
    return 0;
  }

  /**
   * Get the current acceleration in the Z direction.
   *
   * @return the current acceleration in the Z direction
   */
  public double getAccelZ() {
    return 0;
  }

  /**
   * Get the current gyro angle in the X direction.
   *
   * @return the current gyro angle in the X direction
   */
  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  /**
   * Get the current gyro angle in the Y direction.
   *
   * @return the current gyro angle in the Y direction
   */
  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  /**
   * Get the current gyro angle in the Z direction.
   *
   * @return the current gyro angle in the Z direction
   */
  public double getGyroAngleZ() {
    return gyro.getAngleZ();
  }

  /**
   * Get the current gyro rotation as a {@link Rotation2d}.
   *
   * @return the current gyro rotation
   */
  public Rotation2d getGyroRotation2d() {
    double angleZ = getGyroAngleZ();
    double radianZ = Math.toRadians(angleZ);
    Rotation2d rotation2d = new Rotation2d(radianZ);
    SmartDashboard.putNumber("Angle Gyro from Pose", rotation2d.getDegrees());
    return rotation2d;
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Update the pose estimator with vision data
   *
   * @param robotPose the robot pose
   * @param imageCaptureTime the image capture time
   * @param stdVector the standard deviation vector
   */
  public abstract void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Vector<N3> stdVector);

  /** Update the pose estimator with local data */
  public abstract void updateLocalMeasurements();

  /**
   * Get the current pose.
   *
   * @return the current pose
   */
  public abstract Pose2d getCurrentPose();

  /**
   * Reset the pose estimator to a specified pose.
   *
   * @param pose the pose to reset to
   */
  public abstract void resetToPose(Pose2d pose);
}
