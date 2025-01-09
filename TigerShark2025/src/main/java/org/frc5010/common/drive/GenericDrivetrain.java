// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import static edu.wpi.first.units.Units.Kilogram;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.commands.DefaultDriveCommand;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayBoolean;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Generic class for defining drivetrain behavior */
public abstract class GenericDrivetrain extends GenericSubsystem {
  /** The visual representation of the drivetrain */
  protected Mechanism2d mechVisual;

  /** The pose estimator */
  protected DrivePoseEstimator poseEstimator;
  /** Whether or not the robot is field oriented */
  protected DisplayBoolean isFieldOrientedDrive;
  /**
   * Load the RobotConfig from the GUI settings. You should probably
   * store this in your Constants file
   */
  protected RobotConfig config;

  /**
   * Constructor
   *
   * @param mechVisual - The visual representation of the drivetrain
   */
  public GenericDrivetrain(Mechanism2d mechVisual) {
    super(mechVisual);
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // A default config in case the GUI settings can't be loaded
      config = new RobotConfig(Kilogram.of(68).magnitude(),
          SingleJointedArmSim.estimateMOI(0.5, Kilogram.of(68).magnitude()),
          new ModuleConfig(0.1, 4.5, 1.19, 
          DCMotor.getNEO(1), 40, 4), 0.5);
    }

    isFieldOrientedDrive = new DisplayBoolean(true, "Field Oriented", logPrefix, LogLevel.COMPETITION);
  }

  /**
   * Sets the pose estimator for the drivetrain.
   *
   * @param poseEstimator the new pose estimator to be set
   */
  public void setDrivetrainPoseEstimator(DrivePoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  /**
   * Returns the DrivePoseEstimator object associated with this GenericDrivetrain.
   *
   * @return the DrivePoseEstimator object
   */
  public DrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  /**
   * Returns the Mechanism2d object associated with this GenericDrivetrain.
   *
   * @return the Mechanism2d object
   * @throws NullPointerException if the poseEstimator is null
   */
  public Mechanism2d getMechVisual() {
    assert (null != poseEstimator);
    return mechVisual;
  }

  /**
   * Returns the current heading of the pose estimator.
   *
   * @return the current heading as a Rotation2d object
   * @throws AssertionError if the pose estimator is null
   */
  public Rotation2d getHeading() {
    assert (null != poseEstimator);
    return poseEstimator.getGyroRotation2d();
  }

  /**
   * Drive with ChassisSpeeds
   *
   * @param direction vector defining the direction and speed
   */
  public abstract void drive(ChassisSpeeds direction, DriveFeedforwards feedforwards);

  /** Updates the pose estimator in the periodic function. */
  @Override
  public void periodic() {
    poseEstimator.update();
  }

  /** Set the auto builder */
  public abstract void setAutoBuilder();

  /** Called when the robot is disabled */
  public void disabledBehavior() {
  }

  /** Toggles the field oriented drive mode */
  public void toggleFieldOrientedDrive() {
    isFieldOrientedDrive.setValue(!isFieldOrientedDrive.getValue());
  }

  /** Resets the orientation of the pose estimator */
  public void resetOrientation() {
    poseEstimator.resetToPose(
        new Pose2d(
            poseEstimator.getCurrentPose().getTranslation(),
            new Rotation2d(GenericRobot.getAlliance() == Alliance.Blue ? 0 : Math.PI)));
  }

  /** Locks the wheels */
  public void lockWheels() {
  }

  /**
   * Creates a default command for the GenericDrivetrain.
   *
   * @param driver the controller used to control the drivetrain
   * @return a DefaultDriveCommand instance that drives the drivetrain based on
   *         the controller
   *         inputs
   */
  public Command createDefaultCommand(Controller driver) {
    return new DefaultDriveCommand(
        this,
        () -> driver.getLeftYAxis(),
        () -> driver.getLeftXAxis(),
        () -> driver.getRightXAxis(),
        () -> isFieldOrientedDrive.getValue());
  }

  /**
   * Creates a default test command for the GenericDrivetrain.
   *
   * @param driver the controller used to control the drivetrain
   * @return a DefaultDriveCommand instance that drives the drivetrain based on
   *         the controller
   *         inputs
   */
  public Command createDefaultTestCommand(Controller driver) {
    return new DefaultDriveCommand(
        this,
        () -> driver.getLeftYAxis(),
        () -> driver.getLeftXAxis(),
        () -> driver.getRightXAxis(),
        () -> isFieldOrientedDrive.getValue());
  }

  /**
   * Checks if the GenericDrivetrain has any issues.
   *
   * @return false if the GenericDrivetrain does not have any issues, true
   *         otherwise.
   */
  public boolean hasIssues() {
    return false;
  }

  /** Resets the encoders */
  public void resetEncoders() {
  }

  /**
   * Generates an auto command that resets the encoders before starting and
   * continues until the
   * GenericDrivetrain has issues.
   *
   * @param autoCommand the command to be executed
   * @return the generated auto command
   */
  public Command generateAutoCommand(Command autoCommand) {
    if (CommandScheduler.getInstance().isComposed(autoCommand)) {
      return autoCommand;
    } else {
      return autoCommand
        .beforeStarting(
            () -> {
              resetEncoders();
            })
        .until(() -> hasIssues());
    }
    
  }
}
