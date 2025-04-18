// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.pose.SwervePose;
import org.frc5010.common.mechanisms.DriveConstantsDef;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class SwerveDrivetrain extends GenericDrivetrain {

  private ChassisSpeeds chassisSpeeds;

  protected GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

  private GenericGyro gyro;

  private GenericDrivetrainConstants swerveConstants;

  private boolean ready = false;
  private Persisted<Double> maxChassisVelocity;
  private Command defaultDriveCommand;

  public SwerveDrivetrain(
      Mechanism2d mechVisual,
      GenericSwerveModule frontLeft,
      GenericSwerveModule frontRight,
      GenericSwerveModule backLeft,
      GenericSwerveModule backRight,
      GenericGyro genericGyro,
      AprilTagPoseSystem visonSystem,
      SwerveConstants swerveConstants) {
    super(mechVisual);
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;

    this.swerveConstants = swerveConstants;

    this.gyro = genericGyro;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    poseEstimator = new DrivePoseEstimator(
        new SwervePose(gyro, swerveConstants.getKinematics(), this), visonSystem);
    maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);

    setDrivetrainPoseEstimator(poseEstimator);

    gyro.reset();
  }

  public SwerveDrivetrain(
      Mechanism2d mechVisual, GenericGyro genericGyro, SwerveConstants swerveConstants) {
    super(mechVisual);
    this.gyro = genericGyro;
    this.swerveConstants = swerveConstants;
    gyro.reset();
  }

  public SwerveDrivetrain(Mechanism2d mechVisual, GenericDrivetrainConstants swerveConstants) {
    super(mechVisual);
    this.swerveConstants = swerveConstants;
  }

  public Command getDefaultCommand() {
    return defaultDriveCommand;
  }

  public void setupDefaultCommands(Controller driver, Controller operator) {
    // Handle real or simulation case for default commands
    if (RobotBase.isReal()) {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = createDefaultCommand(driver);
        setDefaultCommand(defaultDriveCommand);
      }
    } else {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = createDefaultCommand(driver);
        setDefaultCommand(defaultDriveCommand);
      }
    }
  }

  /**
   * Setup the default test commands
   *
   * @param driver   - driver
   * @param operator - operator
   */
  public void setupTestDefaultCommands(Controller driver, Controller operator) {
    if (RobotBase.isReal()) {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = createDefaultTestCommand(driver);
        setDefaultCommand(defaultDriveCommand);
      }
    } else {
      if (defaultDriveCommand == null) {
        this.defaultDriveCommand = createDefaultCommand(driver);
        setDefaultCommand(defaultDriveCommand);
      }
    }
  }

  /**
   * Drive Axis - Left X and Y, Right X Reset Orientation - Start Lock Wheels -
   * Left Bumper Field
   * Oriented - B Button
   */
  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    // If there needs to be some commands that are real or simulation only use this
    if (!DriverStation.isTest()) {
      if (RobotBase.isReal()) {
        driver.createStartButton().onTrue(Commands.runOnce(() -> resetOrientation()));
      } else {

      }
      driver.createXButton().onTrue(Commands.runOnce(() -> toggleFieldOrientedDrive()));
    }
  }

  @Override
  public void drive(ChassisSpeeds direction, DriveFeedforwards feedforwards) {
    chassisSpeeds = direction; // for driving in simulation
    // Pose2d robotPoseVel = new Pose2d(direction.vxMetersPerSecond * 0.02,
    // direction.vyMetersPerSecond * 0.02,
    // Rotation2d.fromRadians(direction.omegaRadiansPerSecond * 0.02));
    // Twist2d twistVel = getPoseEstimator().getCurrentPose().log(robotPoseVel);
    // chassisSpeeds = new ChassisSpeeds(twistVel.dx / 0.02, twistVel.dy / 0.02,
    // twistVel.dtheta / 0.02);
    SwerveModuleState[] states = ((SwerveConstants) swerveConstants).getKinematics()
        .toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(
            frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
        new SwerveModulePosition(
            frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        new SwerveModulePosition(
            backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        new SwerveModulePosition(
            backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()))
    };
  }

  public double[] getDriveModuleRotations() {
    return new double[] {
      0, 0, 0, 0
    };
  }

  public void setModuleStates(SwerveModuleState[] setDesiredStates) {
    SwerveModuleState[] states = setDesiredStates;
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxChassisVelocity.get());

    // TODO get swerve stop lock working
    // if(Math.abs(states[0].speedMetersPerSecond) < 0.001){
    // states[0] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*5)));
    // states[1] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*7)));
    // states[2] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*3)));
    // states[3] = new SwerveModuleState(0, new
    // Rotation2d(Units.degreesToRadians(45*5)));
    // }

    // TODO add the isReady function so wheels won't move till turned close enough
    boolean isReady = true;
    isReady &= frontLeft.setState(states[0], ready);
    isReady &= frontRight.setState(states[1], ready);
    isReady &= backLeft.setState(states[2], ready);
    isReady &= backRight.setState(states[3], ready);
    ready = isReady;
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveConstants getSwerveConstants() {
    return ((SwerveConstants) swerveConstants);
  }

  public double getGyroRate() {
    return gyro.getRate();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  @Override
  public void simulationPeriodic() {
    Pose2d pose = poseEstimator.getCurrentPose();
    Transform2d direction = new Transform2d(
        new Translation2d(
            chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond * 0.02),
        new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
    pose = pose.transformBy(direction);
    poseEstimator.resetToPose(pose);
  }

  public void setAutoBuilder() {
    AutoBuilder.configure(
        () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
        (Pose2d pose) -> getPoseEstimator().resetToPose(pose), // Pose2d consumer, used to reset odometry at the
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive,
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

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public Command createDefaultCommand(Controller driverXbox, DoubleSupplier rotationSupplier) {
    // System.out.println("brrrrr");
    DoubleSupplier leftX = () -> driverXbox.getLeftXAxis();
    DoubleSupplier leftY = () -> driverXbox.getLeftYAxis();

    return new JoystickToSwerve(
        this,
        leftY,
        leftX,
        rotationSupplier,
        () -> isFieldOrientedDrive.getValue(),
        () -> GenericRobot.getAlliance());
    // return new TeleopDrive(this, leftX, leftY, rightX, isFieldOriented);
  }

  public void disabledBehavior() {
    frontLeft.resetAbsoluteEncoder();
    frontRight.resetAbsoluteEncoder();
    backLeft.resetAbsoluteEncoder();
    backRight.resetAbsoluteEncoder();
  }
}
