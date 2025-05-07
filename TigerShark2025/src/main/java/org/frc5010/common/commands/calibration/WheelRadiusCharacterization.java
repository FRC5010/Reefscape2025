// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands.calibration;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayLength;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelRadiusCharacterization extends GenericCommand {

  private DisplayValuesHelper displayValuesHelper = new DisplayValuesHelper("Wheel Characterization", logPrefix);
  private DisplayDouble wheelRadius;
  private DisplayDouble trackRadius;
  private DisplayDouble currentConfigWheelRadius;
  private DisplayLength wheelDiameter;


  private final double RAMP_RATE = 1.0;
  private final double ROTATION_SPEED = 2.0;
  private SlewRateLimiter rampEnforcer = new SlewRateLimiter(RAMP_RATE);
  private GenericSwerveDrivetrain drivetrain;
  private Supplier<Rotation2d> gyroOverride = null;
  private double robotRotationRadius = 0.0;
  private Timer time = new Timer();

  private final Time dataCollectionDelay = Seconds.of(1);

  private boolean firstCycle = true;

  // Wheel States
  private double[] wheelPositions = new double[4];
  private Rotation2d lastAngle = Rotation2d.kZero;
  private double gyroChange;

  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(GenericSwerveDrivetrain swerve) {
    wheelRadius = displayValuesHelper.makeDisplayDouble("Wheel Radius");
    trackRadius = displayValuesHelper.makeDisplayDouble("Track Radius");
    currentConfigWheelRadius = displayValuesHelper.makeDisplayDouble("Current Config Wheel Radius");
    wheelDiameter = displayValuesHelper.makeDisplayLength("Wheel Diameter");
    
    
    drivetrain = swerve;
    robotRotationRadius = getRotationRadius();
    trackRadius.setValue(robotRotationRadius);
    currentConfigWheelRadius.setValue(drivetrain.getSwerveConstants().getWheelDiameter() / 2.0);

    addRequirements(drivetrain);
  }

  private double getRotationRadius() {
    double swerveWidth = drivetrain.getSwerveConstants().getTrackWidth().in(Meters);
    double swerveHeight = drivetrain.getSwerveConstants().getWheelBase().in(Meters);

    return Math.sqrt(Math.pow(swerveWidth / 2.0, 2) + Math.pow(swerveHeight / 2.0, 2));
  }

  private double[] getCurrentModulePositions() {
    SwerveModulePosition[] modules = drivetrain.getModulePositions();
    double[] positions = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].distanceMeters / (drivetrain.getSwerveConstants().getWheelDiameter() / 2.0);
    }
    return positions;
  }

  private void refreshCurrentModulePositions() {
      wheelPositions = getCurrentModulePositions();
  }

  private void refreshCurrentModulePositions(double[] positions) {
    wheelPositions = positions;
  }



  // Called when the command is initially scheduled.
  @Override
  public void init() {
    rampEnforcer.reset(0.0);

    firstCycle = true;
    gyroChange = 0.0;
    lastAngle = Rotation2d.kZero;

    time.reset();
    time.start();
  }

  private void rotateRobot(double speed) {
    double slewedSpeed = rampEnforcer.calculate(speed);
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, slewedSpeed));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotateRobot(ROTATION_SPEED);
    if (time.hasElapsed(dataCollectionDelay.in(Seconds))) {
      

      Rotation2d currentRotation;
      if (null != gyroOverride) {
        currentRotation = gyroOverride.get();
      } else {
        currentRotation = drivetrain.getHeading();
      }


      double[] currentPositions = getCurrentModulePositions();

      if (firstCycle) {
        refreshCurrentModulePositions(currentPositions);
        lastAngle = currentRotation;
        firstCycle = false;
      }

      double moduleMovement = 0.0;
      for (int i = 0; i < currentPositions.length; i++) {
        moduleMovement += Math.abs(currentPositions[i] - wheelPositions[i]) / 4.0;
      }

      gyroChange += Math.abs(currentRotation.minus(lastAngle).getRadians());
      lastAngle = currentRotation;

      double radius = (gyroChange * robotRotationRadius) / moduleMovement; 
      wheelRadius.setValue(radius);
      wheelDiameter.setLength(Meters.of(radius * 2.0));
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    firstCycle =true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
