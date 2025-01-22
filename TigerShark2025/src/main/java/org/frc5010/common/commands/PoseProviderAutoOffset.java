// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import java.util.function.Supplier;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.GenericDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseProviderAutoOffset extends GenericCommand {
  GenericDrivetrain drivetrain;

  /** Creates a new PoseProviderAutoOffset. */
  public PoseProviderAutoOffset(Supplier<Pose2d> poseSupplier, GenericDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  private Translation2d calculateOffset(Pose2d currentPose) {
    double distance_from_zero = currentPose.getTranslation().getNorm();

    double offset_circle_radius = (distance_from_zero / 2) / currentPose.getRotation().div(2).getSin();
    double triangle_angle = (Math.PI/2) - currentPose.getTranslation().getAngle().getRadians();
    double offset_angle = Math.PI/2 - (triangle_angle + (Math.PI/2 - currentPose.getRotation().div(2).getRadians()));

    double x_shift = Math.sin(offset_angle) * offset_circle_radius;
    double y_shift = Math.cos(offset_angle) * offset_circle_radius;
    Translation2d offset = new Translation2d(x_shift - currentPose.getX(), y_shift - currentPose.getY());

    return offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spin robot slowly
    drivetrain.drive(new ChassisSpeeds(0, 0, 0.314), null);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
