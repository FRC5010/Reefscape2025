// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import org.frc5010.common.arch.Persisted;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.SwervePose;
import org.frc5010.common.mechanisms.DriveConstantsDef;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveModule;

/** Add your docs here. */
public class SwerveDrivetrain extends SwerveDriveFunctions {

  private ChassisSpeeds chassisSpeeds;

  protected GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

  private GenericGyro gyro;

  private GenericDrivetrainConstants swerveConstants;

  private boolean ready = false;
  private Persisted<Double> maxChassisVelocity;
  private DrivePoseEstimator poseEstimator;

  public SwerveDrivetrain(
      LoggedMechanism2d mechVisual,
      GenericSwerveModule frontLeft,
      GenericSwerveModule frontRight,
      GenericSwerveModule backLeft,
      GenericSwerveModule backRight,
      GenericGyro genericGyro,
      SwerveConstants swerveConstants) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.swerveConstants = swerveConstants;

    this.gyro = genericGyro;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    poseEstimator = new DrivePoseEstimator(
        new SwervePose(gyro, swerveConstants.getKinematics(), this));
    maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);

    gyro.reset();
  }

  public SwerveDrivetrain(
      LoggedMechanism2d mechVisual, GenericGyro genericGyro, SwerveConstants swerveConstants) {
    this.gyro = genericGyro;
    this.swerveConstants = swerveConstants;
    gyro.reset();
  }

  public SwerveDrivetrain(LoggedMechanism2d mechVisual, GenericDrivetrainConstants swerveConstants) {
    this.swerveConstants = swerveConstants;
  }

    public DrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  @Override
  public void drive(ChassisSpeeds direction, DriveFeedforwards feedforwards) {
    chassisSpeeds = direction; // for driving in simulation
    // Pose2d robotPoseVel = new Pose2d(direction.vxMetersPerSecond * 0.02,
    // direction.vyMetersPerSecond * 0.02,
    // Rotation2d.fromRadians(direction.omegaRadiansPerSecond * 0.02));
    // Twist2d twistVel = getPoseEstimator().getCurrentPose().log(robotPoseVel);
    // chassisSpeeds = new ChassisSpeeds(twistVel.dx / 0.02, twistVel.dy / 0.02,
    // twistVel.dtheta / 0.02);
    SwerveModuleState[] states = ((SwerveConstants) swerveConstants).getKinematics()
        .toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(
            frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
        new SwerveModulePosition(
            frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        new SwerveModulePosition(
            backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        new SwerveModulePosition(
            backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
    };
  }

  public double[] getDriveModuleRotations() {
    return new double[] {
      0, 0, 0, 0
    };
  }

  public void setModuleStates(SwerveModuleState[] setDesiredStates) {
    SwerveModuleState[] states = setDesiredStates;
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxChassisVelocity.get());

    // TODO get swerve stop lock working
    // if(Math.abs(states[0].speedMetersPerSecond) < 0.001){
    // states[0] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*5)));
    // states[1] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*7)));
    // states[2] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*3)));
    // states[3] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*5)));
    // }

    // TODO add the isReady function so wheels won't move till turned close enough
    boolean isReady = true;
    isReady &= frontLeft.setState(states[0], ready);
    isReady &= frontRight.setState(states[1], ready);
    isReady &= backLeft.setState(states[2], ready);
    isReady &= backRight.setState(states[3], ready);
    ready = isReady;
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveConstants getSwerveConstants() {
    return ((SwerveConstants) swerveConstants);
  }

  public double getGyroRate() {
    return gyro.getRate();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  public void disabledBehavior() {
    frontLeft.resetAbsoluteEncoder();
    frontRight.resetAbsoluteEncoder();
    backLeft.resetAbsoluteEncoder();
    backRight.resetAbsoluteEncoder();
  }

  @Override
  public SwerveModule[] getModules() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getModules'");
  }

  @Override
  public Field2d getField2d() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getField2d'");
  }

  @Override
  public ChassisSpeeds getRobotVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotVelocity'");
  }

  @Override
  public ChassisSpeeds getFieldVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getFieldVelocity'");
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveFieldOriented'");
  }

  @Override
  public void driveRobotRelative(ChassisSpeeds velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveRobotRelative'");
  }

  @Override
  public Command sysIdDriveMotorCommand(SubsystemBase swerveSubsystem) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdDriveMotorCommand'");
  }

  @Override
  public Command sysIdAngleMotorCommand(SubsystemBase swerveSubsystem) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sysIdAngleMotorCommand'");
  }

  @Override
  public void drive(ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'drive'");
  }

  @Override
  public Pose2d getPose() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPose'");
  }

  @Override
  public SwerveModuleState[] getStates() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getStates'");
  }

  @Override
  public AngularVelocity getMaximumModuleAngleVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMaximumModuleAngleVelocity'");
  }

  @Override
  public DrivePoseEstimator initializePoseEstimator() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'initializePoseEstimator'");
  }
}
