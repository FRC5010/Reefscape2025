// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefscapeButtonBoard;
import frc.robot.subsystems.ElevatorSystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;
import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/**
 * A command that will automatically drive the robot to a particular position
 */
public class DriveToPoseSupplierWithElevator extends GenericCommand {
  /** The subsystem that this command will run on */
  private SwerveDrivetrain swerveSubsystem;

  private DisplayValuesHelper displayValuesHelper = new DisplayValuesHelper("PID Values", logPrefix);
  private DisplayDouble rotationkP;
  private DisplayDouble rotationkD;

  private final ProfiledPIDController thetaController;
  private final GenericPID pidRotation = new GenericPID(8.0, 0, 0);
  private final TrapezoidProfile.Constraints thetaConstraints;

  private int onTargetCounter = 0;

  /** The target pose */
  private Pose2d targetPose;

  private Pose2d centerOfReef = new Pose2d();
  private boolean goingToReef = false;

  /** The target transform */
  private Transform2d targetTransform;

  /** The robot pose provider */
  private Supplier<Pose2d> poseProvider;

  /** The target pose provider */
  private Supplier<Pose2d> targetPoseProvider;

  private Supplier<ChassisSpeeds> velocitySupplier = () -> new ChassisSpeeds();
  private Translation2d currentVelocity = new Translation2d(), forwardVector = new Translation2d(), rightVector = new Translation2d(), pathVector = new Translation2d();
  private Command pidElevator;
  private Command stopElevator;

  private ElevatorSystem elevator;

  private final double maxAngularSpeed = 3.14;
  private final double MAX_ANGULAR_ACCELERATION = 6.0;
  private final double MAX_ORDINAL_ACCELERATION = 6.0; // TO-DO: Change
  private final double MAX_ORDINAL_VELOCITY = 4.0;
  private final double MAX_VELOCITY_DEVIATION = 0.1;
  private final double MAX_DISTANCE_DEVIATION = 0.1;
  private final double MIN_VELOCITY = 0.2;
  private final double MIN_FF_RADIUS = 0.05;
  private final double MAX_FF_RADIUS = 0.1;
  private final double VERTICAL_TOLERANCE = 0.03;
  private final double HORIZONTAL_TOLERANCE = 0.03;

  private double forwardA = 0.0, forwardB = 0.0, forwardC = 0.0, backwardA = 0.0, backwardB = 0.0, backwardC = 0.0, horizontalA = 0.0, horizontalB = 0.0, horizontalC = 0.0, verticalA = 0.0, verticalB = 0.0, verticalC = 0.0;
  private double horizontalElevatorEngagementDistance = 0.0, verticalElevatorEngagementDistance = 0.0, horizontalStartBrakingDistance = 0.0, verticalStartBrakingDistance = 0.0, horizontalElevatorVelocityChange = 0.0, verticalElevatorVelocityChange = 0.0, elevatorAverageVelocity = 0.0, maxHorizontalAcceleration = 0.0, maxVerticalAcceleration = 0.0, elevatorLowerBoundTime = 0.0, elevatorExtensionTime = 0.0, elevatorVerticalExtensionTime = 0.0, elevatorHorizontalExtensionTime = 0.0, initialHeight = 0.0, finalHeight = 0.0, horizontalDistance = 0.0, verticalDistance = 0.0; // Everything is in meters and seconds
  private double verticalVelocity = 0.0, horizontalVelocity = 0.0, thetaSpeed = 0.0, previousTime = 0.0, deltaTime = 0.0, ffInclusionFactor = 0.0, previousVerticalVelocity = 0.0, previousHorizontalVelocity = 0.0, overallVelocityMultiplier = 0.0;
  private double initialHorizontalDistance = 0.0;
  private boolean maxVerticalDecelerationEngaged = false, maxHorizontalDecelerationEngaged = false;
  private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

  /**
   * Creates a new DriveToPosition command.
   *
   * @param swerveSubsystem    The drivetrain subsystem
   * @param poseProvider       The pose provider
   * @param targetPoseProvider The target pose provider
   * @param offset             The offset of the target pose
   */
  public DriveToPoseSupplierWithElevator(
      SwerveDrivetrain swerveSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> targetPoseProvider,
      Transform2d offset, ElevatorSystem elevator, Supplier<ChassisSpeeds> robotRelativeSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = (SwerveDrivetrain) swerveSubsystem;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;
    this.elevator = elevator;
    this.velocitySupplier = robotRelativeSpeedSupplier;

    thetaConstraints = new TrapezoidProfile.Constraints(maxAngularSpeed, MAX_ANGULAR_ACCELERATION);
    thetaController = new ProfiledPIDController(pidRotation.getkP(), pidRotation.getkI(), pidRotation.getkD(), thetaConstraints);
    thetaController.setTolerance(Units.degreesToRadians(2));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    rotationkP = displayValuesHelper.makeConfigDouble("Rotation kP");
    rotationkD = displayValuesHelper.makeConfigDouble("Rotation kD");
    if (rotationkP.getValue() == 0) {
      rotationkP.setValue(pidRotation.getkP());
    }
    if (rotationkD.getValue() == 0) {
      rotationkD.setValue(pidRotation.getkD());
    }

    // Setting up directional acceleration constraints
    forwardA = elevator.getForwardAccelerationConstants()[0];
    forwardB = elevator.getForwardAccelerationConstants()[1];
    forwardC = elevator.getForwardAccelerationConstants()[2];
    backwardA = elevator.getBackwardAccelerationConstants()[0];
    backwardB = elevator.getForwardAccelerationConstants()[1];
    backwardC = elevator.getForwardAccelerationConstants()[2];
    horizontalA = elevator.getHorizontalAccelerationConstants()[0];
    horizontalB = elevator.getHorizontalAccelerationConstants()[1];
    horizontalC = elevator.getHorizontalAccelerationConstants()[2];

    targetTransform = offset;

    stopElevator = elevator.stopElevator();

    addRequirements(swerveSubsystem);
  }

  public Translation2d getVectorToTarget() {
    Transform2d toTarget = targetPoseProvider.get().minus(poseProvider.get());
    return new Translation2d(toTarget.getX() / toTarget.getTranslation().getNorm(),
        toTarget.getY() / toTarget.getTranslation().getNorm());
  }

  public double getDistanceToTarget() {
    return targetPoseProvider.get().getTranslation().getDistance(poseProvider.get().getTranslation());
  }

  public Rotation2d getAngleToTarget() {
    return new Translation2d(targetPoseProvider.get().getX() - poseProvider.get().getX(),
        targetPoseProvider.get().getY() - poseProvider.get().getY()).getAngle().plus(Rotation2d.k180deg);
  }

  private void updateTargetPose(Pose2d pose) {
    targetPose = pose;
    thetaController.setGoal(targetPose.getRotation().getRadians());
    swerveSubsystem.getPoseEstimator().setTargetPoseOnField(targetPose, "Target Pose");
  }

  public double calculateElevatorVelocityChange(double a, double b, double c, double elevatorExtensionTime) {
    return elevatorExtensionTime == 0.0 ? 0.0 : ((a * (Math.pow((elevatorAverageVelocity * elevatorExtensionTime) + initialHeight, b + 1) - Math.pow((elevatorAverageVelocity * elevatorLowerBoundTime) + initialHeight, b + 1))) / (elevatorAverageVelocity * (b + 1))) + (c * (elevatorExtensionTime - elevatorLowerBoundTime));
  }

  public double calculateElevatorEngagementDistance(double a, double b, double c, double elevatorExtensionTime) {
    return elevatorExtensionTime == 0.0 ? 0.0 : ((a * (Math.pow((elevatorAverageVelocity * elevatorExtensionTime) + initialHeight, b + 2) - Math.pow((elevatorAverageVelocity * elevatorLowerBoundTime) + initialHeight, b + 2))) / (Math.pow(elevatorAverageVelocity, 2) * (b + 1) * (b + 2))) + ((c / 2) * (Math.pow(elevatorExtensionTime, 2) - Math.pow(elevatorLowerBoundTime, 2)));
  }

  public double calculateStartBrakingDistance(double ordinalVelocity, double elevatorVelocityChange, double elevatorEngagementDistance) {
    return ((Math.pow(elevatorVelocityChange, 2) - Math.pow(ordinalVelocity, 2)) / (-2 * MAX_ORDINAL_ACCELERATION)) + elevatorEngagementDistance;
  }

  public void updateVitalCalculations() {
    verticalElevatorVelocityChange = calculateElevatorVelocityChange(verticalA, verticalB, verticalC, elevatorVerticalExtensionTime);
    horizontalElevatorVelocityChange = calculateElevatorVelocityChange(horizontalA, horizontalB, horizontalC, elevatorHorizontalExtensionTime);
    verticalElevatorEngagementDistance = calculateElevatorEngagementDistance(verticalA, verticalB, verticalC, elevatorVerticalExtensionTime);
    horizontalElevatorEngagementDistance = calculateElevatorEngagementDistance(horizontalA, horizontalB, horizontalC, elevatorHorizontalExtensionTime);
    verticalStartBrakingDistance = calculateStartBrakingDistance(verticalVelocity, verticalElevatorVelocityChange, verticalElevatorEngagementDistance);
    horizontalStartBrakingDistance = calculateStartBrakingDistance(horizontalVelocity, horizontalElevatorVelocityChange, horizontalElevatorEngagementDistance);
  }

  public void engageMaximumAcceleration() { // TO-DO: Consider adding a fix instead of max distance deviation
    if (verticalStartBrakingDistance > Math.abs(verticalDistance) - 2 * (Math.abs(verticalVelocity) * deltaTime) && (Math.signum(verticalVelocity) == Math.signum(verticalDistance) || verticalVelocity == 0)) {
      verticalVelocity -= Math.signum(verticalDistance) * (MAX_ORDINAL_ACCELERATION * deltaTime);
    } else {
      verticalVelocity += Math.signum(verticalDistance) * (MAX_ORDINAL_ACCELERATION * deltaTime);
    }

    if (horizontalStartBrakingDistance > Math.abs(horizontalDistance) - 2 * (Math.abs(horizontalVelocity) * deltaTime) && (Math.signum(horizontalVelocity) == Math.signum(horizontalDistance) || horizontalVelocity == 0)) {
      horizontalVelocity -= Math.signum(horizontalDistance) * (MAX_ORDINAL_ACCELERATION * deltaTime);
    } else {
      horizontalVelocity += Math.signum(horizontalDistance) * (MAX_ORDINAL_ACCELERATION * deltaTime);
    }

    limitVelocity();

    SmartDashboard.putBoolean("Maximum Acceleration Engaged", true);
  }

  public double getMaximumAcceleration(double a, double b, double c) {
    return (a * Math.pow(elevator.getElevatorPosition().in(Meters), b)) + c;
  }

  public void limitVelocityToVelocityCurve() {
    if (Math.abs(verticalVelocity) + (maxVerticalAcceleration * deltaTime) > verticalElevatorVelocityChange) {
      verticalVelocity = Math.signum(verticalDistance) * verticalElevatorVelocityChange;
    } else {
      verticalVelocity += Math.signum(verticalDistance) * (maxVerticalAcceleration * deltaTime);
    }
    if (Math.abs(horizontalVelocity) + (maxHorizontalAcceleration * deltaTime) > horizontalElevatorEngagementDistance) {
      horizontalVelocity = Math.signum(horizontalDistance) * horizontalElevatorVelocityChange;
    } else {
      horizontalVelocity += Math.signum(horizontalDistance) * (maxHorizontalAcceleration * deltaTime);
    }
    verticalVelocity = Math.signum(verticalDistance) * Math.max(Math.abs(verticalVelocity), MIN_VELOCITY);
    horizontalVelocity = Math.signum(horizontalDistance) * Math.max(Math.abs(horizontalVelocity), MIN_VELOCITY);
    limitVelocity();
    elevatorLowerBoundTime += deltaTime;
  }

  public void limitVerticalVelocityToVelocityCurve() {
    if (Math.abs(verticalVelocity) + (maxVerticalAcceleration * deltaTime) > verticalElevatorVelocityChange) {
      verticalVelocity = Math.signum(verticalDistance) * verticalElevatorVelocityChange;
    } else {
      verticalVelocity += Math.signum(verticalDistance) * (maxVerticalAcceleration * deltaTime);
    }
    verticalVelocity = Math.signum(verticalDistance) * Math.max(Math.abs(verticalVelocity), MIN_VELOCITY);
    limitVelocity();
  }

  public void limitHorizontalVelocityToVelocityCurve() {
    if (Math.abs(horizontalVelocity) + (maxHorizontalAcceleration * deltaTime) > horizontalElevatorEngagementDistance) {
      horizontalVelocity = Math.signum(horizontalDistance) * horizontalElevatorVelocityChange;
    } else {
      horizontalVelocity += Math.signum(horizontalDistance) * (maxHorizontalAcceleration * deltaTime);
    }
    horizontalVelocity = Math.signum(horizontalDistance) * Math.max(Math.abs(horizontalVelocity), MIN_VELOCITY);
    limitVelocity();
  }

  private void limitVelocity() {
    verticalVelocity = Math.signum(verticalVelocity) * Math.min(Math.abs(verticalVelocity), MAX_ORDINAL_VELOCITY);
    horizontalVelocity = Math.signum(horizontalVelocity) * Math.min(Math.abs(horizontalVelocity), MAX_ORDINAL_VELOCITY);
    overallVelocityMultiplier = Math.min(MAX_ORDINAL_VELOCITY / Math.sqrt(Math.pow(horizontalVelocity, 2) + Math.pow(verticalVelocity, 2)), 1.0);
    verticalVelocity *= overallVelocityMultiplier;
    horizontalVelocity *= overallVelocityMultiplier;
  }

  public boolean verticalControllerAtTarget() {
    return Math.abs(verticalDistance) < VERTICAL_TOLERANCE;
  }

  public boolean horizontalControllerAttarget() {
    return Math.abs(horizontalDistance) < HORIZONTAL_TOLERANCE;
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    if (null != targetPoseProvider.get()) {
      updateTargetPose(targetPoseProvider.get().transformBy(targetTransform));
    }

    Pose2d robotPose = poseProvider.get();
    onTargetCounter = 0;
    currentVelocity = new Translation2d(velocitySupplier.get().vxMetersPerSecond, velocitySupplier.get().vyMetersPerSecond).rotateBy(targetPose.getRotation().minus(robotPose.getRotation()));
    thetaController.reset(robotPose.getRotation().getRadians(), velocitySupplier.get().omegaRadiansPerSecond);

    thetaController.setP(rotationkP.getValue());
    thetaController.setD(rotationkD.getValue()); 
    
    centerOfReef = DriverStation.getAlliance().get() == Alliance.Blue ? new Pose2d(4.48945, 4.0259, new Rotation2d()) : new Pose2d(13.065, 4.0259, new Rotation2d());
    goingToReef = targetPoseProvider.get().getTranslation().getDistance(centerOfReef.getTranslation()) < 2.0;
    verticalA = goingToReef ? backwardA : forwardA;
    verticalB = goingToReef ? backwardB : forwardB;
    verticalC = goingToReef ? backwardC : forwardC;
    elevator.setControlType(PIDControlType.NONE); // TO-DO: Eventually take unnecessary logic out
    finalHeight = elevator.selectElevatorLevel(() -> ReefscapeButtonBoard.getScoringLevel()).in(Meters);
    elevatorAverageVelocity = elevator.getAverageElevatorVelocity();
    elevatorExtensionTime = elevator.getElevatorExtensionTime();
    elevatorVerticalExtensionTime = elevatorExtensionTime;
    elevatorHorizontalExtensionTime = elevatorExtensionTime;  
    initialHeight = elevator.getElevatorPosition().in(Meters);
    elevatorLowerBoundTime = elevatorAverageVelocity == 0.0 ? 0.0 : (elevator.getElevatorPosition().in(Meters) - initialHeight) / elevatorAverageVelocity;
    verticalVelocity = currentVelocity.getX();
    horizontalVelocity = currentVelocity.getY();
    updateVitalCalculations();
    maxHorizontalDecelerationEngaged = false;
    maxVerticalDecelerationEngaged = false;
    previousHorizontalVelocity = 0.0;
    previousVerticalVelocity = 0.0;
    pidElevator = elevator.pidControlCommand(Meters.of(finalHeight));

    previousTime = System.nanoTime() / 1E9;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose2d = poseProvider.get();
    Pose2d providedTargetPose = targetPoseProvider.get();
    if (null != providedTargetPose) {
      Pose2d target = providedTargetPose;
      updateTargetPose(target); // Add conditional to update the target pose
    }

    thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians());
    currentVelocity = new Translation2d(velocitySupplier.get().vxMetersPerSecond, velocitySupplier.get().vyMetersPerSecond).rotateBy(targetPose.getRotation().minus(robotPose2d.getRotation()));
    forwardVector = new Translation2d(1.0, 0.0).rotateBy(providedTargetPose.getRotation());
    rightVector = new Translation2d(0.0, 1.0).rotateBy(providedTargetPose.getRotation()); // TO-DO: Ensure this code rotates the vectors correctly
    pathVector = targetPose.getTranslation().minus(robotPose2d.getTranslation());
    ffInclusionFactor = MathUtil.clamp((pathVector.getNorm() - MIN_FF_RADIUS) / (MAX_FF_RADIUS - MIN_FF_RADIUS), 0.0, 1.0);
    verticalDistance = (forwardVector.getX() * pathVector.getX()) + (forwardVector.getY() * pathVector.getY());
    SmartDashboard.putNumber("Psuedo Vertical Distance", pathVector.getX());
    horizontalDistance = (rightVector.getX() * pathVector.getX()) + (rightVector.getY() * pathVector.getY());
    maxVerticalAcceleration = getMaximumAcceleration(verticalA, verticalB, verticalC);
    maxHorizontalAcceleration = getMaximumAcceleration(horizontalA, horizontalB, horizontalC);

    deltaTime = (System.nanoTime() / 1E9) - previousTime;
    previousTime = System.nanoTime() / 1E9;

    boolean verticalTooCloseElevator = Math.abs(verticalDistance) < verticalElevatorEngagementDistance;
    boolean verticalTooFastElevator = Math.abs(verticalVelocity - (verticalElevatorVelocityChange + (maxVerticalAcceleration * deltaTime))) > MAX_VELOCITY_DEVIATION;
    boolean verticalEnsureRightDirection = Math.abs(verticalVelocity) > verticalElevatorVelocityChange + (maxVerticalAcceleration * deltaTime);
    boolean horizontalTooCloseElevator = Math.abs(horizontalDistance) < horizontalElevatorEngagementDistance;
    boolean horizontalTooFastElevator = Math.abs(horizontalVelocity - (horizontalElevatorVelocityChange + (maxHorizontalAcceleration * deltaTime))) > MAX_VELOCITY_DEVIATION;
    boolean horizontalEnsureRightDirection = Math.abs(horizontalVelocity) > horizontalElevatorEngagementDistance + (maxHorizontalAcceleration * deltaTime);

    if ((verticalTooCloseElevator && verticalTooFastElevator && verticalEnsureRightDirection) || (horizontalTooCloseElevator && horizontalTooFastElevator && horizontalEnsureRightDirection)) {
      elevatorExtensionTime -= deltaTime; // TO-DO: Consider reseting the extension time upon stopping the elevator
      elevatorVerticalExtensionTime = Math.abs(verticalDistance) < verticalElevatorEngagementDistance ? elevatorVerticalExtensionTime - deltaTime : elevatorVerticalExtensionTime;
      elevatorHorizontalExtensionTime = Math.abs(horizontalDistance) < horizontalElevatorEngagementDistance ? elevatorHorizontalExtensionTime - deltaTime : elevatorHorizontalExtensionTime;
      if (!stopElevator.isScheduled()) {
        stopElevator.schedule();
      }
      engageMaximumAcceleration();
    }  else if (Math.abs(verticalDistance) < verticalElevatorEngagementDistance || Math.abs(horizontalDistance) < horizontalElevatorEngagementDistance) {
      if (Math.abs(verticalDistance) < verticalElevatorEngagementDistance && Math.abs(horizontalDistance) < horizontalElevatorEngagementDistance)
        if (!pidElevator.isScheduled()) {
          pidElevator.schedule();
        }
      if (Math.abs(verticalDistance) < verticalElevatorEngagementDistance) {
        limitVerticalVelocityToVelocityCurve();
      }
      if (Math.abs(horizontalDistance) < horizontalElevatorEngagementDistance) {
        limitHorizontalVelocityToVelocityCurve();
      }
      elevatorLowerBoundTime += deltaTime;
      SmartDashboard.putBoolean("Maximum Acceleration Engaged", false);
    } else {
      engageMaximumAcceleration();
    }
    updateVitalCalculations();
    verticalVelocity = verticalControllerAtTarget() && (Math.abs(verticalVelocity) < 0.25 || previousVerticalVelocity == 0.0) ? 0.0 : verticalVelocity;
    horizontalVelocity = horizontalControllerAttarget() && (Math.abs(horizontalVelocity) < 0.25 || previousHorizontalVelocity == 0.0) ? 0.0 : horizontalVelocity;
    previousVerticalVelocity = verticalVelocity;
    previousHorizontalVelocity = horizontalVelocity;

    initialHorizontalDistance = initialHorizontalDistance == 0 ? horizontalDistance : initialHorizontalDistance - (horizontalVelocity * deltaTime);
    SmartDashboard.putNumber("Psuedo Horizontal Distance", initialHorizontalDistance);

    // robotChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(verticalVelocity, horizontalVelocity, thetaSpeed + (thetaController.getSetpoint().velocity * ffInclusionFactor), swerveSubsystem.getHeading());
    // robotChassisSpeeds = new ChassisSpeeds(verticalVelocity, horizontalVelocity, thetaSpeed + (thetaController.getSetpoint().velocity * ffInclusionFactor));
    Translation2d fieldRelativeChassisSpeeds = new Translation2d(verticalVelocity, horizontalVelocity).rotateBy(providedTargetPose.getRotation());
    robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(fieldRelativeChassisSpeeds.getX(), fieldRelativeChassisSpeeds.getY(), thetaSpeed), robotPose2d.getRotation());
    // robotChassisSpeeds = new ChassisSpeeds(verticalVelocity, 0.0, 0.0);

    SmartDashboard.putNumber("Vertical Velocity", verticalVelocity);
    SmartDashboard.putNumber("Actual Vertical Velocity", currentVelocity.getX());
    SmartDashboard.putNumber("Horizontal Velocity", horizontalVelocity);
    SmartDashboard.putNumber("Vertical Distance", verticalDistance);
    SmartDashboard.putNumber("Horizontal Distance", horizontalDistance);
    SmartDashboard.putNumber("Vertical Start Braking Distance", verticalStartBrakingDistance);
    SmartDashboard.putNumber("Horizontal Start Braking Distance", horizontalStartBrakingDistance);
    SmartDashboard.putNumber("Vertical Elevator Engagement Distance", verticalElevatorEngagementDistance);
    SmartDashboard.putNumber("Horizontal Elevator Engagement Distance", horizontalElevatorEngagementDistance);
    SmartDashboard.putNumber("Elevator Extension Time", elevatorExtensionTime);
    SmartDashboard.putNumber("Theta Speed", robotChassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Path Vector X", pathVector.getX());
    SmartDashboard.putNumber("Path Vector Y", pathVector.getY());
    SmartDashboard.putNumber("Forward Vector X", forwardVector.getX());
    SmartDashboard.putNumber("Right Vector Y", rightVector.getY());
    SmartDashboard.putBoolean("Vertical Controller At Setpoint", verticalControllerAtTarget());
    SmartDashboard.putBoolean("Horizontal Controller At Setpoint", horizontalControllerAttarget());
    SmartDashboard.putBoolean("Theta Controller at Setpoint", thetaController.atGoal());
    swerveSubsystem.drive(robotChassisSpeeds, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    onTargetCounter = 0;
    SmartDashboard.putBoolean("DriveToPositionWithElevatorInterrupted", interrupted);
    swerveSubsystem.drive(
        new ChassisSpeeds(0, 0, 0), null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (verticalControllerAtTarget() && horizontalControllerAttarget() && elevator.isAtTarget().getAsBoolean()) {
      onTargetCounter++;
    }
    return onTargetCounter > 5;
  }
}
