// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.YAGSLSwervePose;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
public class YAGSLSwerveDrivetrain extends SwerveDrivetrain {
  /** Swerve drive object. */
  private static SwerveDrive swerveDrive = null;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(19.5);

  /** 5010 Code */
  private DoubleSupplier angleSpeedSupplier = null;
  private DisplayBoolean hasIssues;

  public YAGSLSwerveDrivetrain(
      Mechanism2d mechVisual,
      GenericDrivetrainConstants constants,
      double kTurningMotorGearRatio,
      String swerveType) {
    super(mechVisual, constants);
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
          new Pose2d(new Translation2d(Meter.of(7.2),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
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
    SwerveConstants swerveConstants = (SwerveConstants) constants;
    if (swerveConstants.getSwerveModuleConstants().getDriveFeedForward().size() > 0) {
      Map<String, MotorFeedFwdConstants> motorFFMap = swerveConstants.getSwerveModuleConstants().getDriveFeedForward();
      Map<String, SwerveModule> swerveModuleMap = swerveDrive.getModuleMap();
      motorFFMap.keySet().stream()
          .forEach(
              module -> {
                MotorFeedFwdConstants ff = motorFFMap.get(module);
                double kS = ff.getkS();
                double kV = ff.getkV();
                double kA = ff.getkA();
                swerveModuleMap.get(module).setFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
              });
    }

    setDrivetrainPoseEstimator(new DrivePoseEstimator(new YAGSLSwervePose(this)));

    SmartDashboard.putString(
        "YAGSL Alliance", GenericRobot.chooseAllianceDisplayColor().toString());
    hasIssues = new DisplayBoolean(false, "Has Issues", logPrefix, LogLevel.COMPETITION);
    if (RobotBase.isSimulation() || useGlass) {
      initGlassWidget();
    }
    if (RobotBase.isSimulation()) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
      int count = 0;
      for (Pose3d gpa : SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceA)) {
        getField2d().getObject("CARPET" + count).setPose(new Pose2d(gpa.getX(), gpa.getY(), new Rotation2d()));
        getField2d().getObject("GPA" + count).setPose(new Pose2d(gpa.getX(), gpa.getY(), gpa.getRotation().toRotation2d()));
        count++;
      }  
      count = 0;
      for (Pose3d gpb : SimulatedArena.getInstance().getGamePiecesByType(Constants.Simulation.gamePieceB)) {
        getField2d().getObject("GPB" + count).setPose(new Pose2d(gpb.getX(), gpb.getY(), gpb.getRotation().toRotation2d()));
        count++;
      }  
    }
  }

  /** Setup AutoBuilder for PathPlanner. */
  public void setupPathPlanner() {
    AutoBuilder.configure(
        poseEstimator::getCurrentPose, // Robot pose supplier
        poseEstimator::resetToPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeedsWithAngleSupplier,
        // RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = GenericRobot.getAlliance();
          SmartDashboard.putString(
              "YAGSL Alliance", alliance.toString());
          return alliance == DriverStation.Alliance.Red;
        },
        this // Reference to this subsystem to set requirements
    );

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
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
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
        getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond(),
        getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
        getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());
    // PathConstraints constraints = new PathConstraints(
    // swerveDrive.getMaximumChassisVelocity(), 4.0,
    // swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
   * PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
   *                                  achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */

  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new SysIdRoutine.Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
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
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
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
    swerveDrive.drive(velocity);
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
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
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

  @Override
  public void resetOrientation() {
    poseEstimator.resetToPose(poseEstimator.getCurrentPose());
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

  /** 5010 Code */
  @Override
  public void periodic() {
    hasIssues.setValue(hasIssues());
    if (RobotBase.isSimulation() || useGlass) {
      updateGlassWidget();
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


  public void setAngleSupplier(DoubleSupplier angDoubleSupplier) {
    angleSpeedSupplier = angDoubleSupplier;
  }

  public void setChassisSpeedsWithAngleSupplier(ChassisSpeeds chassisSpeeds, DriveFeedforwards moduleFeedForwards) {
    ChassisSpeeds angleSuppliedChassisSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        null != angleSpeedSupplier
            ? angleSpeedSupplier.getAsDouble()
            : chassisSpeeds.omegaRadiansPerSecond);
    swerveDrive.drive(
        angleSuppliedChassisSpeeds,
        swerveDrive.kinematics.toSwerveModuleStates(angleSuppliedChassisSpeeds),
        moduleFeedForwards.linearForces());
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return getFieldVelocity();
  }

  /** 5010 Code */
  public Command createDefaultCommand(Controller driverXbox) {
    DoubleSupplier leftX = () -> driverXbox.getAxisValue(XboxController.Axis.kLeftX.value);
    DoubleSupplier leftY = () -> driverXbox.getAxisValue(XboxController.Axis.kLeftY.value);
    DoubleSupplier rightX = () -> driverXbox.getAxisValue(XboxController.Axis.kRightX.value);
    BooleanSupplier isFieldOriented = () -> isFieldOrientedDrive.getValue();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDrive,
    //     leftY,
    //     leftX)
    //     .withControllerRotationAxis(rightX)
    //     .deadband(0.07)
    //     .scaleTranslation(0.8)
    //     .allianceRelativeControl(true);

    // return driveFieldOriented(driveAngularVelocity);
    return new JoystickToSwerve(
        this, leftY, leftX, rightX, isFieldOriented, () -> GenericRobot.getAlliance());
  }

  public Command createDefaultTestCommand(Controller driverXbox) {
    DoubleSupplier leftX = () -> driverXbox.getAxisValue(XboxController.Axis.kLeftX.value);
    DoubleSupplier leftY = () -> driverXbox.getAxisValue(XboxController.Axis.kLeftY.value);
    DoubleSupplier rightX = () -> driverXbox.getAxisValue(XboxController.Axis.kRightX.value);
    BooleanSupplier isFieldOriented = () -> isFieldOrientedDrive.getValue();

    // driverXbox.createAButton().whileTrue(sysIdDriveMotorCommand());
    // driverXbox.createBButton().whileTrue(sysIdAngleMotorCommand());
    // return Commands.run(() -> SwerveDriveTest.centerModules(swerveDrive), this);
    return new JoystickToSwerve(
        this, leftY, leftX, rightX, isFieldOriented, () -> GenericRobot.getAlliance());
  }

  @Override
  public void stop() {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void resetEncoders() {
    swerveDrive.resetDriveEncoders();
    swerveDrive.synchronizeModuleEncoders();
  }

  @Override
  public double getGyroRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);
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
  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  @Override
  public void disabledBehavior() {
  }

  public void setAutoBuilder() {
    setupPathPlanner();
  }

  private int issueCheckCycles = 0;
  private int issueCount = 0;
  private static boolean useGlass = false;
  private Map<Integer, MechanismRoot2d> visualRoots = new HashMap<>();
  private Map<Integer, MechanismLigament2d> motorDials = new HashMap<>();
  private Map<Integer, MechanismLigament2d> absEncDials = new HashMap<>();
  private Map<Integer, MechanismLigament2d> expectDials = new HashMap<>();

  private int badConnections = 0;
  private double lowLimit = Units.inchesToMeters(-1);
  private double highXLimit = Units.feetToMeters(54);
  private double highYLimit = Units.feetToMeters(27);

  @Override
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
        System.err.println(
            "********************************CAN is being flakey********************************");
      }
      if (issueCount > 5) {
        System.err.println(
            "********************************Robot position is off field********************************");
      }

      return badConnections > 5 || !positionOk;
    }
    return false;
  }

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]^T,
   * with units in meters
   * and radians.
   */
  public void updateVisionMeasurements(
      Pose2d robotPose, double imageCaptureTime, Vector<N3> stdVector) {
    swerveDrive.addVisionMeasurement(robotPose, imageCaptureTime, stdVector);
  }

  public Pose2d getMapleSimPose() {
    return swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose();
  }

  public Field2d getField2d() {
    return swerveDrive.field;
  }

  public static void useGlass(boolean shouldUseGlass) {
    useGlass = shouldUseGlass;
  }

  public void initGlassWidget() {
    SmartDashboard.putData("Drive Visual", mechanismSimulation);
    SwerveModule[] modules = swerveDrive.getModules();

    visualRoots.put(
        0,
        mechanismSimulation.getRoot(
            "frontleft",
            RobotConstantsDef.robotVisualH * modules[0].configuration.moduleLocation.getX()
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * modules[0].configuration.moduleLocation.getY()
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        1,
        mechanismSimulation.getRoot(
            "frontright",
            RobotConstantsDef.robotVisualH * modules[1].configuration.moduleLocation.getX()
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * modules[1].configuration.moduleLocation.getY()
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        2,
        mechanismSimulation.getRoot(
            "backleft",
            RobotConstantsDef.robotVisualH * modules[2].configuration.moduleLocation.getX()
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * modules[2].configuration.moduleLocation.getY()
                + RobotConstantsDef.robotVisualV / 2.0));
    visualRoots.put(
        3,
        mechanismSimulation.getRoot(
            "backright",
            RobotConstantsDef.robotVisualH * modules[3].configuration.moduleLocation.getX()
                + RobotConstantsDef.robotVisualH / 2.0,
            RobotConstantsDef.robotVisualV * modules[3].configuration.moduleLocation.getY()
                + RobotConstantsDef.robotVisualV / 2.0));
    for (int i = 0; i < modules.length; i++) {
      visualRoots
          .get(i)
          .append(new MechanismLigament2d(i + "-vert", 0.10, 90, 6.0, new Color8Bit(50, 50, 50)));
      visualRoots
          .get(i)
          .append(new MechanismLigament2d(i + "-hori", 0.10, 0, 6.0, new Color8Bit(50, 50, 50)));
      motorDials.put(
          i,
          visualRoots
              .get(i)
              .append(
                  new MechanismLigament2d(
                      i + "-motor", 0.10, 90, 6.0, new Color8Bit(Color.kYellow))));
      absEncDials.put(
          i,
          visualRoots
              .get(i)
              .append(
                  new MechanismLigament2d(i + "-Abs", 0.10, 90, 6, new Color8Bit(Color.kBlue))));
      expectDials.put(
          i,
          visualRoots
              .get(i)
              .append(new MechanismLigament2d(i + "-Exp", 0.10, 90, 6, new Color8Bit(Color.kRed))));
    }
  }

  public void updateGlassWidget() {
    SwerveModule[] modules = swerveDrive.getModules();
    for (int moduleKey = 0; moduleKey < modules.length; moduleKey++) {
      double turningDeg = modules[moduleKey].getRelativePosition();
      double absEncDeg = modules[moduleKey].getAbsolutePosition();
      // This method will be called once per scheduler run
      absEncDials.get(moduleKey).setAngle(absEncDeg + 90);
      motorDials.get(moduleKey).setAngle(turningDeg + 90);
      motorDials
          .get(moduleKey)
          .setLength(0.0001 * modules[moduleKey].getAngleMotor().getVelocity() + 0.002);
      expectDials
          .get(moduleKey)
          .setLength(0.0001 * modules[moduleKey].getDriveMotor().getVelocity() + 0.002);
      expectDials.get(moduleKey).setAngle(modules[moduleKey].getState().angle.getDegrees() + 90);
    }
  }
}
