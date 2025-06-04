// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import static edu.wpi.first.units.Units.Kilogram;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.commands.DefaultDriveCommand;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Generic class for defining drivetrain behavior */
public abstract class GenericDrivetrain extends GenericSubsystem {
  /** The pose estimator */
  protected DrivePoseEstimator poseEstimator;
  /** Whether or not the robot is field oriented */
  protected DisplayBoolean isFieldOrientedDrive;
  /**
   * Load the RobotConfig from the GUI settings. You should probably
   * store this in your Constants file
   */
  protected RobotConfig config;
  protected DisplayBoolean hasIssues;
  protected DoubleSupplier angleSpeedSupplier = null;
  public Supplier<Double> maxForwardAcceleration, maxBackwardAcceleration, maxLeftAcceleration, maxRightAcceleration;
  public Supplier<Double> maxForwardVelocity, maxBackwardVelocity, maxLeftVelocity, maxRightVelocity;
  protected Pose2d obstaclePosition = new Pose2d();
  protected Pose2d[] unavoidableVertices = new Pose2d[0];
  protected double obstacleRadius = 0.0, robotRadius = 0.0, maxRobotDimensionDeviation = 0.0,
      maxObstacleDimensionDeviation = 0.0;
  protected int obstacleAvoidanceResolution = 0;
  protected double previousLeftXInput = 0.0, previousLeftYInput = 0.0, previousRightXInput = 0.0;
  protected Alert canErrorAlert = new Alert("CAN Tx/Rx is being FLAKY!", AlertType.kError);
  protected Alert robotPositionAlert = new Alert("Robot position is off field", AlertType.kError);
  /**
   * Constructor
   *
   * @param mechVisual - The visual representation of the drivetrain
   */
  public GenericDrivetrain(LoggedMechanism2d mechVisual) {
    super(mechVisual);
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // A default config in case the GUI settings can't be loaded
      config = new RobotConfig(Kilogram.of(68).magnitude(),
          SingleJointedArmSim.estimateMOI(0.5, Kilogram.of(68).magnitude()),
          new ModuleConfig(0.1, 4.5, 1.19,
              DCMotor.getNEO(1), 40, 4),
          0.5);
    }

    isFieldOrientedDrive = displayValues.makeDisplayBoolean("Field Oriented Drive");
    isFieldOrientedDrive.setValue(true);
    hasIssues = new DisplayBoolean(false, "Has Issues", logPrefix, LogLevel.COMPETITION);
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
   * Returns the LoggedMechanism2d object associated with this GenericDrivetrain.
   *
   * @return the LoggedMechanism2d object
   * @throws NullPointerException if the poseEstimator is null
   */
  public LoggedMechanism2d getMechVisual() {
    assert (null != poseEstimator);
    return mechanismSimulation;
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
  public abstract void driveWithFeedforwards(ChassisSpeeds direction, DriveFeedforwards feedforwards);

  /**
   * Drive with ChassisSpeeds
   *
   * @param direction vector defining the direction and speed
   */
  public abstract void drive(ChassisSpeeds direction);

  /** Updates the pose estimator in the periodic function. */
  @Override
  public void periodic() {
    hasIssues.setValue(hasIssues());
    if (RobotBase.isSimulation() || useGlass) {
      updateGlassWidget();
    }
  }

  /** Set the auto builder */
  public abstract void setAutoBuilder();

  /** Called when the robot is disabled */
  public void disabledBehavior() {
  }

  /** Called when the configuring the button bindings */
  public void configureButtonBindings(Controller driver, Controller operator) {
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

              })
          .until(() -> hasIssues());
    }

  }

  @Override
  public void simulationPeriodic() {
    int count = 0;
    List<Pose3d> gpas = SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceA);
    for (Pose3d gpa : gpas) {
      getField2d().getObject("GPA" + count++).setPose(
          new Pose2d(gpa.getX(), gpa.getY(), gpa.getRotation().toRotation2d()));
    }
    count = 0;
    List<Pose3d> gpbs = SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceB);
    for (Pose3d gpb : gpbs) {
      getField2d().getObject("GPB" + count++).setPose(
          new Pose2d(gpb.getX(), gpb.getY(), gpb.getRotation().toRotation2d()));
    }
  }

  protected void initializeSimulation() {
    if (RobotBase.isSimulation() || useGlass) {
      initGlassWidget();
    }
    if (RobotBase.isSimulation()) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
      int count = 0;
      for (Pose3d gpa : SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceA)) {
        getField2d().getObject("CARPET" + count).setPose(new Pose2d(gpa.getX(), gpa.getY(), new Rotation2d()));
        getField2d().getObject("GPA" + count)
            .setPose(new Pose2d(gpa.getX(), gpa.getY(), gpa.getRotation().toRotation2d()));
        count++;
      }
      count = 0;
      for (Pose3d gpb : SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceB)) {
        getField2d().getObject("GPB" + count)
            .setPose(new Pose2d(gpb.getX(), gpb.getY(), gpb.getRotation().toRotation2d()));
        count++;
      }
    }
  }

  public abstract Field2d getField2d();

  public void setAccelerationSuppliers(Supplier<Double> maxForwardAcceleration,
      Supplier<Double> maxBackwardAcceleration, Supplier<Double> maxLeftAcceleration,
      Supplier<Double> maxRightAcceleration) {
    this.maxForwardAcceleration = maxForwardAcceleration;
    this.maxBackwardAcceleration = maxBackwardAcceleration;
    this.maxLeftAcceleration = maxLeftAcceleration;
    this.maxRightAcceleration = maxRightAcceleration;
  }

  public void setVelocitySuppliers(Supplier<Double> maxForwardVelocity, Supplier<Double> maxBackwardVelocity,
      Supplier<Double> maxRightVelocity, Supplier<Double> maxLeftVelocity) {
    this.maxForwardVelocity = maxForwardVelocity;
    this.maxBackwardVelocity = maxBackwardVelocity;
    this.maxRightVelocity = maxRightVelocity;
    this.maxLeftVelocity = maxLeftVelocity;
  }

  public void setUpCircularObstacle(Pose2d obstaclePosition, Pose2d[] unavoidableVertices, double obstacleRadius,
      double robotRadius, double maxRobotDimensionDeviation, double maxObstacleDimensionDeviation,
      int resolution) {
    this.obstaclePosition = obstaclePosition;
    this.unavoidableVertices = unavoidableVertices;
    this.obstacleRadius = obstacleRadius;
    this.robotRadius = robotRadius;
    this.maxRobotDimensionDeviation = maxRobotDimensionDeviation;
    this.maxObstacleDimensionDeviation = maxObstacleDimensionDeviation;
    this.obstacleAvoidanceResolution = resolution;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetToPose(pose);
  }

  protected int issueCheckCycles = 0;
  protected int issueCount = 0;
  protected static boolean useGlass = false;
  protected Map<Integer, LoggedMechanismRoot2d> visualRoots = new HashMap<>();
  protected Map<Integer, LoggedMechanismLigament2d> motorDials = new HashMap<>();
  protected Map<Integer, LoggedMechanismLigament2d> absEncDials = new HashMap<>();
  protected Map<Integer, LoggedMechanismLigament2d> expectDials = new HashMap<>();

  protected int badConnections = 0;
  protected double lowLimit = Units.inchesToMeters(-1);
  protected double highXLimit = Units.feetToMeters(54);
  protected double highYLimit = Units.feetToMeters(27);

  public void initGlassWidget() {
  }

  public void updateGlassWidget() {
  }

  public static void useGlass(boolean shouldUseGlass) {
    useGlass = shouldUseGlass;
  }

  /**
   * Checks if the GenericDrivetrain has any issues.
   *
   * @return false if the GenericDrivetrain does not have any issues, true
   *         otherwise.
   */
  public boolean hasIssues() {

    issueCheckCycles++;
    if (issueCheckCycles > 10) {
      issueCheckCycles = 0;

      boolean doesCanHaveIssues = RobotController.getCANStatus().transmitErrorCount
          + RobotController.getCANStatus().receiveErrorCount > 0;

      Translation2d currTranslation = getPoseEstimator().getCurrentPose().getTranslation();
      boolean positionOk = !DriverStation.isAutonomous()
          || (currTranslation.getX() >= lowLimit && currTranslation.getY() >= lowLimit)
              && (currTranslation.getX() <= highXLimit && currTranslation.getY() <= highYLimit)
              && (!Double.isNaN(currTranslation.getX())
                  && !Double.isNaN(currTranslation.getY()));

      if (doesCanHaveIssues) {
        badConnections++;
      } else {
        badConnections = 0;
      }
      if (!positionOk) {
        issueCount++;
      } else {
        issueCount = 0;
      }

      if (badConnections > 5) {
        canErrorAlert.set(true);
      } else {
        canErrorAlert.set(false);
      }
      if (issueCount > 5) {
        robotPositionAlert.set(true);
      } else {
        robotPositionAlert.set(false);
      }

      return badConnections > 5 || !positionOk;
    }
    return false;
  }

}
