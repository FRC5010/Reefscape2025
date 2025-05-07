// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartSwerveDrive extends Command {
  private GenericSwerveDrivetrain swerveDrive;

  private DoubleSupplier xSpdFunction, ySpdFunction, turnSpdFunction;
  private BooleanSupplier fieldOrientedDrive;
  private Supplier<Alliance> allianceSupplier;

  /** Creates a new SmartDrive. */
  public SmartSwerveDrive(
    GenericSwerveDrivetrain swerveSubsystem,
      DoubleSupplier xSpdFunction,
      DoubleSupplier ySpdFunction,
      DoubleSupplier turnSpdFunction,
      BooleanSupplier fieldOrientedDrive,
      Supplier<Alliance> allianceSupplier) {
    this.swerveDrive = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;
    this.fieldOrientedDrive = fieldOrientedDrive;
    this.allianceSupplier = allianceSupplier;

    addRequirements(this.swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void setTurnSpeedFunction(DoubleSupplier turnSpeedFunction) {
    turnSpdFunction = turnSpeedFunction;
  }

  public DoubleSupplier getTurnSpeedFunction() {
    return turnSpdFunction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get values on sticks and deadzone them

    double xInput = (xSpdFunction.getAsDouble());
    double yInput = (ySpdFunction.getAsDouble());

    Translation2d inputTranslation = new Translation2d(xInput, yInput);
    double magnitude = inputTranslation.getNorm();
    Rotation2d angle = 0 != xInput || 0 != yInput ? inputTranslation.getAngle() : new Rotation2d();

    double curvedMagnitude = Math.pow(magnitude, 3);

    double turnSpeed = (turnSpdFunction.getAsDouble());

    // limit power
    double xSpeed =
        curvedMagnitude
            * angle.getCos()
            * swerveDrive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    double ySpeed =
        curvedMagnitude
            * angle.getSin()
            * swerveDrive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    turnSpeed =
        turnSpeed * swerveDrive.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    // convert to chassis speed class
    ChassisSpeeds chassisSpeeds;
    //
    // System.out.println(swerveDrive.getGyroRate());
    double gyroRate = Units.degreesToRadians(swerveDrive.getGyroRate()) * 0.01;
    Rotation2d correctedRotation = swerveDrive.getHeading().minus(new Rotation2d(gyroRate));

    if (fieldOrientedDrive.getAsBoolean()) {
      Alliance alliance = allianceSupplier.get();
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(    
              alliance == Alliance.Red ? -xSpeed : xSpeed,
              alliance == Alliance.Red ? -ySpeed : ySpeed,
              turnSpeed, correctedRotation);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    // convert chassis speed into modules speeds
    // SwerveModuleState[] moduleStates =
    // SwerveDrivetrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // output each module speed into subsystem

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
