// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.YAGSLSwervePose;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class YAGSLSwerveDrivetrain extends SwerveDriveFunctions {
  /** Swerve drive object. */
  private static SwerveDrive swerveDrive = null;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(19.5);

  public YAGSLSwerveDrivetrain(
      GenericDrivetrainConstants constants,
      double kTurningMotorGearRatio,
      String swerveType) {
    this.maximumSpeed = constants.getkTeleDriveMaxSpeedMetersPerSecond();

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleGearRatio = 1.0 / kTurningMotorGearRatio;
    double wheelDiameter = constants.getWheelDiameter();
    double driveGearRatio = 1.0 / constants.getkDriveMotorGearRatio();
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio, 1);

    // Motor conversion factor is (PI * WHEEL DIAMETER) / (GEAR RATIO * ENCODER
    // RESOLUTION).
    // In this case the wheel diameter is 4 inches.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(wheelDiameter, driveGearRatio, 1);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    // SwerveDriveTelemetry.verbosity = LogLevel.DEBUG ==
    // GenericRobot.getLoggingLevel() ?
    // TelemetryVerbosity.HIGH : TelemetryVerbosity.INFO;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), swerveType);
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed,
          new Pose2d(new Translation2d(Meter.of(7.417),
              Meter.of(4)),
              Rotation2d.fromDegrees(180)));
    } catch (Exception e) {
      System.out.println(e.getMessage());
      throw new RuntimeException(e);
    }
    // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setHeadingCorrection(false);

    // Disables cosine compensation for simulations since it causes discrepancies
    // not seen in real life.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(true, 3);
    // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders();
    // Set the absolute encoder to be used over the internal encoder and push the
    // offsets onto it. Throws warning if not possible

    /** 5010 Code */
    // SwerveConstants swerveConstants = (SwerveConstants) constants;
    // if (swerveConstants.getSwerveModuleConstants().getDriveFeedForward().size() >
    // 0) {
    // Map<String, MotorFeedFwdConstants> motorFFMap =
    // swerveConstants.getSwerveModuleConstants().getDriveFeedForward();
    // Map<String, SwerveModule> swerveModuleMap = swerveDrive.getModuleMap();
    // motorFFMap.keySet().stream()
    // .forEach(
    // module -> {
    // MotorFeedFwdConstants ff = motorFFMap.get(module);
    // double kS = ff.getkS();
    // double kV = ff.getkV();
    // double kA = ff.getkA();
    // swerveModuleMap.get(module).setFeedforward(new SimpleMotorFeedforward(kS, kV,
    // kA));
    // });
    // }
  }

  public DrivePoseEstimator initializePoseEstimator() {
    return new DrivePoseEstimator(new YAGSLSwervePose(this));
  }

  @Override
  public double getGyroRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);
  }
  /** END 5010 Code */

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  @Override
  public Command sysIdDriveMotorCommand(SubsystemBase swerveSubsystem) {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(7), Second.of(30),
                (state) -> SignalLogger.writeString("state", state.toString())), swerveSubsystem,
            swerveDrive, 12, true),
        3.0, 10.0, 4.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  @Override
  public Command sysIdAngleMotorCommand(SubsystemBase swerveSubsystem) {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            swerveSubsystem, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a
   * given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per
   *                               second
   * @return a Command that drives the swerve drive to a specific distance at a
   *         given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
   * object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  xInput,
                  yInput,
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(
        () -> {
          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble() * Math.PI,
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()));
        });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   * 
   * @param translationX     Translation in the X direction. Cubed for smoother
   * @param translationY     Translation in the Y direction. Cubed for smoother
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Drive the robot using the {@link SwerveModuleState}, it is recommended to
   * have
   * {@link SwerveDrive#setCosineCompensator(boolean)} set to false for this.<br/>
   *
   * @param robotRelativeVelocity Robot relative {@link ChassisSpeeds}
   * @param states                Corresponding {@link SwerveModuleState} to use
   *                              (not checked against the
   *                              {@param robotRelativeVelocity}).
   * @param feedforwardForces     Feedforward forces generated by set-point
   *                              generator
   */
  public void drive(ChassisSpeeds robotRelativeVelocity, SwerveModuleState[] states, Force[] feedforwardForces) {
    swerveDrive.drive(robotRelativeVelocity, states, feedforwardForces);
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either
   * open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and
   * robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in
   *                      meters per second. In robot-relative mode, positive x is
   *                      torwards the bow (front) and
   *                      positive y is torwards port (left). In field-relative
   *                      mode, positive x is away from the
   *                      alliance wall (field North) and positive y is torwards
   *                      the left wall when looking through
   *                      the driver station glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by
   *                      field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity and drive
   * feedforwards.
   *
   * @param velocity     Robot oriented {@link ChassisSpeeds}
   * @param feedforwards {@link DriveFeedforwards}
   */
  @Override
  public void drive(ChassisSpeeds velocity, DriveFeedforwards feedforwards) {
    swerveDrive.drive(
        velocity,
        swerveDrive.kinematics.toSwerveModuleStates(velocity),
        feedforwards.linearForces());
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when
   * calling this method. However, if either gyro angle or module position is
   * reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is
   *         available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be
   * corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, headingX, headingY, getHeading().getRadians(), maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /** Add a fake vision reading for testing purposes. */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
        new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Get the swerve drive object, which has the actual driving logic, encoder
   * data, etc.
   *
   * @return The swerve drive object.
   */
  public static SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void resetEncoders() {
    swerveDrive.resetDriveEncoders();
    swerveDrive.synchronizeModuleEncoders();
  }

  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]^T,
   * with units in meters
   * and radians.
   */
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime,Matrix<N3, N1> stdVector) {
    swerveDrive.addVisionMeasurement(robotPose, imageCaptureTime, stdVector);
  }

/**
 * Retrieves the pose of the simulated drivetrain from the MapleSim system.
 *
 * @return The current pose of the simulated drivetrain as a {@link Pose2d}.
 */

  public Pose2d getMapleSimPose() {
    return swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose();
  }

  public Field2d getField2d() {
    return swerveDrive.field;
  }

  @Override
  public SwerveModule[] getModules() {
    return swerveDrive.getModules();
  }

  @Override
  public void stop() {
    swerveDrive.drive(new ChassisSpeeds());
  }

  @Override
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public SwerveModuleState[] getStates() {
    return swerveDrive.getStates();
  }

  @Override
  public AngularVelocity getMaximumModuleAngleVelocity() {
    return swerveDrive.getMaximumModuleAngleVelocity();
  }

  @Override
  public void driveRobotRelative(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }
}
