// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.subsystems.LedSubsystem;

/**
 * A command that will automatically drive the robot to a particular position
 */
public class DriveToPosition extends GenericCommand {
  /** The subsystem that this command will run on */
  private SwerveDrivetrain swerveSubsystem;
  /** The PID constants for translation */
  private final GenericPID pidTranslation = new GenericPID(1, 0, 0);
  /** The PID constants for rotation */
  private final GenericPID pidRotation = new GenericPID(.25, 0, 0);

  /** The constraints for translation in the X direction */
  private final TrapezoidProfile.Constraints xConstraints;
  /** The constraints for translation in the Y direction */
  private final TrapezoidProfile.Constraints yConstraints;
  /** The constraints for rotation */
  private final TrapezoidProfile.Constraints thetaConstraints;

  /** The PID controller for translation in the X direction */
  private final ProfiledPIDController xController;
  /** The PID controller for translation in the Y direction */
  private final ProfiledPIDController yController;
  /** The PID controller for rotation */
  private final ProfiledPIDController thetaController;

  /** The target pose */
  private Pose2d targetPose;
  /** The target transform */
  private Transform2d targetTransform;

  /** The robot pose provider */
  private Supplier<Pose2d> poseProvider;
  /** The target pose provider */
  private Supplier<Pose3d> targetPoseProvider;

  /** The speed that the robot will drive at in the X direction */
  private double xSpeed;
  /** The speed that the robot will drive at in the Y direction */
  private double ySpeed;
  /** The speed that the robot will rotate at */
  private double thetaSpeed;

  /**
   * Creates a new DriveToPosition command.
   *
   * @param swerveSubsystem    The drivetrain subsystem
   * @param poseProvider       The pose provider
   * @param targetPoseProvider The target pose provider
   * @param ledSubsystem       The LED subsystem
   * @param offset             The offset of the target pose
   */
  public DriveToPosition(
      SwerveDrivetrain swerveSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose3d> targetPoseProvider,
      LedSubsystem ledSubsystem,
      Transform2d offset) {
    xConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond());
    yConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond());
    thetaConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
        swerveSubsystem
            .getSwerveConstants()
            .getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

    xController = new ProfiledPIDController(
        pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(), xConstraints);
    yController = new ProfiledPIDController(
        pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(), yConstraints);
    thetaController = new ProfiledPIDController(
        pidRotation.getkP(), pidRotation.getkI(), pidRotation.getkD(), thetaConstraints);

    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = (SwerveDrivetrain) swerveSubsystem;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetTransform = offset;

    addRequirements(swerveSubsystem);
  }

  private void updateTargetPose(Pose2d pose) {
    targetPose = pose;

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());
    swerveSubsystem.getPoseEstimator().setTargetPoseOnField(targetPose, "Target Pose");
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    Pose2d robotPose = poseProvider.get();

    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    if (null != targetPoseProvider.get()) {
      updateTargetPose(targetPoseProvider.get().toPose2d().transformBy(targetTransform));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();

    Pose3d providedTargetPose = targetPoseProvider.get();
    if (null != providedTargetPose) {
      Pose2d target = providedTargetPose.toPose2d().transformBy(targetTransform);
      if (target.getTranslation().getDistance(targetPose.getTranslation()) < 1.0) {
        updateTargetPose(target);
      }
    }

    // System.out.println(robotPose2d);

    // System.out.println(robotPose);
    xSpeed = xController.calculate(robotPose2d.getX())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    ySpeed = yController.calculate(robotPose2d.getY())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    if (xController.atGoal()) {
      xSpeed = 0;
    }

    if (yController.atGoal()) {
      ySpeed = 0;
    }

    if (thetaController.atGoal()) {
      thetaSpeed = 0;
    }

    // // Transform the tag's pose to set our goal

    // System.out.println(goalPose);
    // SmartDashboard.putNumber("X Speed", xSpeed);
    // SmartDashboard.putNumber("Y Speed", ySpeed);
    // SmartDashboard.putNumber("Theta Speed", thetaSpeed);

    // System.out.println(thetaSpeed);
    // swerveSubsystem.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -thetaSpeed));
    // System.out.println("xSpeed: " + xController.calculate(robotPose.getX()) +
    // "\nySpeed: " + yController.calculate(robotPose.getY()) +
    // "\nTheta: " +
    // thetaController.calculate(robotPose2d.getRotation().getRadians()));

    // swerveSubsystem.drive(new ChassisSpeeds(
    // xController.calculate(robotPose.getX()) *
    // swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
    // yController.calculate(robotPose.getY()) *
    // swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()
    // ,0));

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, thetaSpeed, swerveSubsystem.getHeading());

    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Theta Speed", chassisSpeeds.omegaRadiansPerSecond);
    swerveSubsystem.drive(chassisSpeeds, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    swerveSubsystem.drive(
        new ChassisSpeeds(0, 0, 0), null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xSpeed == 0) && (ySpeed == 0) && (thetaSpeed == 0);
  }
}
