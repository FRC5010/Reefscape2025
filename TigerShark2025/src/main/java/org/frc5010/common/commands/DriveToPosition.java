// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import java.util.function.Supplier;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A command that will automatically drive the robot to a particular position
 */
public class DriveToPosition extends GenericCommand {
  /** The subsystem that this command will run on */
  private GenericSwerveDrivetrain swerveSubsystem;
  /** The PID constants for translation */

  private DisplayValuesHelper displayValuesHelper = new DisplayValuesHelper("PID Values", logPrefix);
  private DisplayDouble translationkP;
  private DisplayDouble translationkD;
  private DisplayDouble rotationkP;
  private DisplayDouble rotationkD;

  private final GenericPID pidTranslation = new GenericPID(0.8, 0, 0);
  /** The PID constants for rotation */
  private final GenericPID pidRotation = new GenericPID(2.0, 0, 0);

  private int onTargetCounter = 0;

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

  private Supplier<ChassisSpeeds> initialVelocity = () -> new ChassisSpeeds();

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
   * @param offset             The offset of the target pose
   */
  public DriveToPosition(
    GenericSwerveDrivetrain swerveSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose3d> targetPoseProvider,
      Transform2d offset) {
    xConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()/Math.sqrt(2),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()/Math.sqrt(2));
    yConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()/Math.sqrt(2),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()/Math.sqrt(2));
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
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetTransform = offset;

    translationkP = displayValuesHelper.makeConfigDouble("Translation kP");
    translationkD = displayValuesHelper.makeConfigDouble("Translation kD");
    rotationkP = displayValuesHelper.makeConfigDouble("Rotation kP");
    rotationkD = displayValuesHelper.makeConfigDouble("Rotation kD");

    if (translationkP.getValue() == 0) {
      translationkP.setValue(pidTranslation.getkP());
    }
    if (translationkD.getValue() == 0) {
      translationkD.setValue(pidTranslation.getkD());
    }
    if (rotationkP.getValue() == 0) {
      rotationkP.setValue(pidRotation.getkP());
    }
    if (rotationkD.getValue() == 0) {
      rotationkD.setValue(pidRotation.getkD());
    }

    addRequirements(swerveSubsystem);
  }

  public DriveToPosition withInitialVelocity(Supplier<ChassisSpeeds> speedSupplier) {
    initialVelocity = speedSupplier;
    return this;
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
    xController.setP(translationkP.getValue());
    xController.setD(translationkD.getValue());
    yController.setP(translationkP.getValue());
    yController.setD(translationkD.getValue());
    thetaController.setP(rotationkP.getValue());
    thetaController.setD(rotationkD.getValue()); 

    Pose2d robotPose = poseProvider.get();
    onTargetCounter = 0;

    thetaController.reset(robotPose.getRotation().getRadians(), initialVelocity.get().omegaRadiansPerSecond);
    xController.reset(robotPose.getX(), initialVelocity.get().vxMetersPerSecond);
    yController.reset(robotPose.getY(), initialVelocity.get().vyMetersPerSecond);

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

    // if (xController.atGoal()) {
    //   xSpeed = 0;
    // }

    // if (yController.atGoal()) {
    //   ySpeed = 0;
    // }

    // if (thetaController.atGoal()) {
    //   thetaSpeed = 0;
    // }

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
    double minFFRadius = 0.05;
    double maxFFRadius = 0.15;
    double distanceToGoal = robotPose2d.getTranslation().getDistance(targetPose.getTranslation());
    double ffInclusionFactor = MathUtil.clamp((distanceToGoal - minFFRadius) / (maxFFRadius - minFFRadius), 0.0, 1.0);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed + xController.getSetpoint().velocity*ffInclusionFactor, ySpeed + yController.getSetpoint().velocity*ffInclusionFactor, thetaSpeed + thetaController.getSetpoint().velocity*ffInclusionFactor, swerveSubsystem.getHeading());

    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Theta Speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("X Controller at Setpoins", xController.atGoal());
    SmartDashboard.putBoolean("Y Controller at Setpoins", yController.atGoal());
    SmartDashboard.putBoolean("Theta Controller at Setpoins", thetaController.atGoal());
    swerveSubsystem.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    onTargetCounter = 0;
    SmartDashboard.putBoolean("DriveToPositionInterrupted", interrupted);
    swerveSubsystem.drive(
        new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (xController.atGoal() && yController.atGoal() && thetaController.atGoal()) {
      onTargetCounter++;
    }

    return onTargetCounter > 20;
  }
}
