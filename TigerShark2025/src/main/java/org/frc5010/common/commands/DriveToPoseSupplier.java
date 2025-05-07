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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A command that will automatically drive the robot to a particular position
 */
public class DriveToPoseSupplier extends GenericCommand {
  /** The subsystem that this command will run on */
  private GenericSwerveDrivetrain swerveSubsystem;
  /** The PID constants for translation */

  private DisplayValuesHelper displayValuesHelper = new DisplayValuesHelper("PID Values", logPrefix);
  private DisplayDouble translationkP;
  private DisplayDouble translationkD;
  private DisplayDouble rotationkP;
  private DisplayDouble rotationkD;

  private final GenericPID pidTranslation = new GenericPID(7, 0, 0.0);
  /** The PID constants for rotation */
  private final GenericPID pidRotation = new GenericPID(7.0, 0, 0);

  private int onTargetCounter = 0;

  /** The constraints for translation in the Y direction */
  private final TrapezoidProfile.Constraints translationConstraints;
  /** The constraints for rotation */
  private final TrapezoidProfile.Constraints thetaConstraints;

  /** The PID controller for translation in the X direction */
  private final ProfiledPIDController distanceController;
  /** The PID controller for rotation */
  private final ProfiledPIDController thetaController;

  /** The target pose */
  private Pose2d targetPose;
  /** The target transform */
  private Transform2d targetTransform;

  /** The robot pose provider */
  private Supplier<Pose2d> poseProvider;
  /** The target pose provider */
  private Supplier<Pose2d> targetPoseProvider;

  private Supplier<ChassisSpeeds> initialVelocity = () -> new ChassisSpeeds();

  private final double maxAngularSpeed = 3.14;
  private final double MAX_ANGULAR_ACCELERATION = 5.0;

  /** The speed that the robot will drive at in the X direction */
  private double translationalSpeed;
  /** The speed that the robot will rotate at */
  private double thetaSpeed;

  private final double MAX_VELOCITY = 4.18;
  private double maxAcceleration = 4.3;

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private double speedTowardsTarget = 0.0;
  private Translation2d currentVelocity = new Translation2d();
  private double previousTime = 0.0, deltaTime = 0.0;
  private final int NUM_CYCLES = 1;

  /**
   * Creates a new DriveToPosition command.
   *
   * @param swerveSubsystem    The drivetrain subsystem
   * @param poseProvider       The pose provider
   * @param targetPoseProvider The target pose provider
   * @param offset             The offset of the target pose
   */
  public DriveToPoseSupplier(
      GenericSwerveDrivetrain swerveSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> targetPoseProvider,
      Transform2d offset, double maxAcceleration) {
    this.maxAcceleration = maxAcceleration;
    translationConstraints = new TrapezoidProfile.Constraints(
        MAX_VELOCITY,
        maxAcceleration);
  
    thetaConstraints = new TrapezoidProfile.Constraints(
        maxAngularSpeed,
        MAX_ANGULAR_ACCELERATION);

    distanceController = new ProfiledPIDController(
        pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(), translationConstraints);
    thetaController = new ProfiledPIDController(
        pidRotation.getkP(), pidRotation.getkI(), pidRotation.getkD(), thetaConstraints);

    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;

    distanceController.setTolerance(0.02);
    thetaController.setTolerance(Units.degreesToRadians(1.0));
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

  public Translation2d getVectorToTarget() {
    Transform2d toTarget = targetPoseProvider.get().minus(poseProvider.get());
    return new Translation2d(toTarget.getX()/toTarget.getTranslation().getNorm(), toTarget.getY()/toTarget.getTranslation().getNorm());
  }

  public double getDistanceToTarget() {
    return targetPoseProvider.get().getTranslation().getDistance(poseProvider.get().getTranslation());
  }

  public double getFutureDistanceToTarget() {
    return targetPoseProvider.get().getTranslation().getDistance(getFuturePosition());
  }

  public Translation2d getFuturePosition() {
    return poseProvider.get().getTranslation().plus(currentVelocity.times(deltaTime * NUM_CYCLES));
  }

  public Rotation2d getAngleToTarget() {
    return new Translation2d(targetPoseProvider.get().getX() - poseProvider.get().getX(), targetPoseProvider.get().getY() - poseProvider.get().getY()).getAngle().plus(Rotation2d.k180deg);
  }

  public Rotation2d getFutureAngleToTarget() {
    Translation2d futurePose = getFuturePosition();
    return new Translation2d(targetPoseProvider.get().getX() - futurePose.getX(), targetPoseProvider.get().getY() - futurePose.getY()).getAngle().plus(Rotation2d.k180deg);
  }

  public DriveToPoseSupplier withInitialVelocity(Supplier<ChassisSpeeds> speedSupplier) {
    initialVelocity = speedSupplier;
    return this;
  }
 
  private void updateTargetPose(Pose2d pose) {
    targetPose = pose;

    distanceController.setGoal(0);
    thetaController.setGoal(targetPose.getRotation().getRadians());
    swerveSubsystem.getPoseEstimator().setTargetPoseOnField(targetPose, "Target Pose");
  }

  private void updateSpeedTowardsTarget() {
    currentVelocity = new Translation2d(initialVelocity.get().vxMetersPerSecond, initialVelocity.get().vyMetersPerSecond);
    speedTowardsTarget = Math.min(
            0.0,
            -currentVelocity
                .rotateBy(
                    targetPoseProvider
                        .get()
                        .getTranslation()
                        .minus(poseProvider.get().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX());
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    distanceController.setP(translationkP.getValue());
    distanceController.setD(translationkD.getValue());

    thetaController.setP(rotationkP.getValue());
    thetaController.setD(rotationkD.getValue()); 

    Pose2d robotPose = poseProvider.get();
    onTargetCounter = 0;

    Translation2d toTarget = getVectorToTarget();
    thetaController.reset(robotPose.getRotation().getRadians(), initialVelocity.get().omegaRadiansPerSecond);
    updateSpeedTowardsTarget();
    
    distanceController.reset(getDistanceToTarget(), speedTowardsTarget);

    lastSetpointTranslation = robotPose.getTranslation();
    lastSetpointRotation = targetPoseProvider.get().getRotation();
    lastTime = Timer.getTimestamp();
  

    if (null != targetPoseProvider.get()) {
      updateTargetPose(targetPoseProvider.get().transformBy(targetTransform));
    }
    
    previousTime = Timer.getFPGATimestamp();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose2d = poseProvider.get();

    Pose2d providedTargetPose = targetPoseProvider.get();
    if (null != providedTargetPose) {
      Pose2d target = providedTargetPose;
      if (target.getTranslation().getDistance(targetPose.getTranslation()) < 1.0) {
        updateTargetPose(target);
      }
    }

    // System.out.println(robotPose2d);

    // System.out.println(robotPose);
    
 
    

 
    double minFFRadius = 0.05;
    double maxFFRadius = 0.1;
    currentVelocity = new Translation2d(initialVelocity.get().vxMetersPerSecond, initialVelocity.get().vyMetersPerSecond);
    double distanceToGoal = getDistanceToTarget();
    double futureDistanceToGoal = getFutureDistanceToTarget();
    double ffInclusionFactor = MathUtil.clamp((distanceToGoal - minFFRadius) / (maxFFRadius - minFFRadius), 0.0, 1.0);
    // distanceController.reset(
    //     lastSetpointTranslation.getDistance(targetPose.getTranslation()),
    //     distanceController.getSetpoint().velocity);
    deltaTime = Timer.getFPGATimestamp() - previousTime;
    previousTime = Timer.getFPGATimestamp();

    double translationalSpeed = distanceToGoal > 0.25 ? distanceController.calculate(futureDistanceToGoal, 0.0) : distanceController.calculate(distanceToGoal, 0.0);
    Rotation2d movementAngle = getAngleToTarget();

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        robotPose2d.getTranslation().getY() - targetPose.getTranslation().getY(),
                        robotPose2d.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(new Transform2d(distanceController.getSetpoint().position, 0.0, new Rotation2d()))
            .getTranslation();
    
    double speedX = movementAngle.getCos() * translationalSpeed;
    double speedY = movementAngle.getSin() * translationalSpeed;



    thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians(), new TrapezoidProfile.State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)));

    lastSetpointRotation = targetPose.getRotation();

    lastTime = Timer.getTimestamp();
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        speedX + distanceController.getSetpoint().velocity*movementAngle.getCos()*ffInclusionFactor, speedY + distanceController.getSetpoint().velocity*movementAngle.getSin()*ffInclusionFactor, thetaSpeed + thetaController.getSetpoint().velocity*Math.min(ffInclusionFactor*5, 1.0), swerveSubsystem.getHeading());

    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Distance Velocity Setpoint", distanceController.getSetpoint().velocity);
    SmartDashboard.putNumber("Theta Speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putBoolean("Distance Controller At Setpoint", distanceController.atGoal());
    SmartDashboard.putNumber("VelocityError", distanceController.getVelocityError());
    SmartDashboard.putNumber("PositionError", distanceController.getPositionError());
    SmartDashboard.putNumber("Theta Error", thetaController.getPositionError());

    SmartDashboard.putBoolean("Theta Controller at Setpoint", thetaController.atGoal());
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
    return distanceController.atGoal() && thetaController.atGoal();
  }
}
