// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import java.util.function.Supplier;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.GenericDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseProviderAutoOffset extends GenericCommand {
  GenericDrivetrain drivetrain;
  Supplier<Pose2d> pose;
  Rotation2d cameraRotation;
  private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
  private int _calculatedOffsetToRobotCenterCount = 0;
  private Pose2d initPose;
  private Rotation2d initRobotRotation;
  private String name = "";


  /** Creates a new PoseProviderAutoOffset. */
  public PoseProviderAutoOffset(Supplier<Pose2d> rawPoseSupplier, GenericDrivetrain drivetrain, Rotation2d rotation, String name) {
    this.drivetrain = drivetrain;
    this.pose = rawPoseSupplier;
    this.cameraRotation = rotation;
    this.name = name;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public PoseProviderAutoOffset(Supplier<Pose2d> rawPoseSupplier, GenericDrivetrain drivetrain, Rotation2d rotation) {
    this(rawPoseSupplier, drivetrain, rotation, "");
  }

  private Translation2d calculateOffsetToRobotCenter() {
        Pose2d currentPose2d = getPose();

        Rotation2d angle = getHeading();
        Translation2d displacement = currentPose2d.getTranslation();

        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
                / (2 * (1 - angle.getCos()));
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
                / (2 * (1 - angle.getCos()));

        return new Translation2d(x, y);
    }

  public Rotation2d getHeading() {
    return drivetrain.getHeading().minus(initRobotRotation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    _calculatedOffsetToRobotCenter = new Translation2d();
    _calculatedOffsetToRobotCenterCount = 0;
    initPose = pose.get();
    initRobotRotation = drivetrain.getHeading();
  }

  public Pose2d getPose() {
    Pose2d raw = pose.get();
    Pose2d zeroOffset = new Pose2d(raw.getTranslation().minus(initPose.getTranslation()), raw.getRotation().minus(initPose.getRotation()));
    Pose2d rotationAccounted = zeroOffset.rotateBy(cameraRotation);
    return rotationAccounted;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spin robot slowly
    drivetrain.drive(new ChassisSpeeds(0, 0, 0.314), null);
    if (null != pose.get()) {
      Rotation2d rotation = getHeading();

      Translation2d offset = calculateOffsetToRobotCenter();

      _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter
              .times((double) _calculatedOffsetToRobotCenterCount
                      / (_calculatedOffsetToRobotCenterCount + 1))
              .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
      _calculatedOffsetToRobotCenterCount++;
    }
    SmartDashboard.putNumberArray(name+" Camera Calculated Offset to Robot Center", new double[] {
            _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY() });
      
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
