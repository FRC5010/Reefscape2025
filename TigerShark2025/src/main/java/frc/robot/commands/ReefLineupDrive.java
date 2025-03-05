// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.drive.swerve_utils.PathConstraints5010;
import org.frc5010.common.drive.swerve_utils.SwerveSetpointGenerator5010;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLineupDrive extends Command  {
  SwerveSetpointGenerator5010 setpointGenerator;
  AtomicReference<SwerveSetpoint> previousSetpoint;
  AtomicReference<Double> previousTime;
  YAGSLSwerveDrivetrain drivetrain;
  SwerveDrive swerveDrive;
  PathConstraints5010 pathConstraints;

  DoubleSupplier xInput;
  DoubleSupplier yInput;
  Angle targetAngle;
  ScoringLocation side;

  private final GenericPID pidRotation = new GenericPID(0.5, 0, 0);

  private final TrapezoidProfile.Constraints thetaConstraints;

  private final ProfiledPIDController thetaController;

  /** Creates a new ReefLineupDrive. */
  public ReefLineupDrive(YAGSLSwerveDrivetrain drivetrain, DoubleSupplier xInput, DoubleSupplier yInput, ScoringLocation side) throws IOException, ParseException {
    this.drivetrain = drivetrain;
    this.side = side;
    this.xInput = xInput;
    this.yInput = yInput;
    swerveDrive = drivetrain.getSwerveDrive();
    setpointGenerator = new SwerveSetpointGenerator5010(RobotConfig.fromGUISettings(), drivetrain.getSwerveDrive().getMaximumModuleAngleVelocity().in(RadiansPerSecond));
    previousSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
            swerveDrive.getStates(),
            DriveFeedforwards.zeros(swerveDrive.getModules().length)));

    thetaConstraints = new TrapezoidProfile.Constraints(
              drivetrain.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
              drivetrain
                  .getSwerveConstants()
                  .getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

    thetaController = new ProfiledPIDController(
        pidRotation.getkP(), pidRotation.getkI(), pidRotation.getkD(), thetaConstraints);

    thetaController.setTolerance(2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    previousTime = new AtomicReference<>();
    // TODO: Don't reference the variables directly, make them private and add getters
    pathConstraints = new PathConstraints5010(
        MetersPerSecond.of(drivetrain.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()),
        MetersPerSecondPerSecond.of(drivetrain.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond()),
        (Supplier<LinearAcceleration>) () -> MetersPerSecondPerSecond.of(drivetrain.maxForwardAcceleration.get()),
        (Supplier<LinearAcceleration>) () -> MetersPerSecondPerSecond.of(drivetrain.maxBackwardAcceleration.get()),
        (Supplier<LinearAcceleration>) () -> MetersPerSecondPerSecond.of(drivetrain.maxRightAcceleration.get()),
        (Supplier<LinearAcceleration>) () -> MetersPerSecondPerSecond.of(drivetrain.maxLeftAcceleration.get()),
        (Supplier<LinearVelocity>) () -> MetersPerSecond.of(drivetrain.maxForwardVelocity.get()),
        (Supplier<LinearVelocity>) () -> MetersPerSecond.of(drivetrain.maxBackwardVelocity.get()),
        (Supplier<LinearVelocity>) () -> MetersPerSecond.of(drivetrain.maxLeftVelocity.get()),
        (Supplier<LinearVelocity>) () -> MetersPerSecond.of(drivetrain.maxRightVelocity.get()),
        RadiansPerSecond.of(drivetrain.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond()),
        RadiansPerSecondPerSecond.of(drivetrain.getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond()),
        Volts.of(12), false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public ChassisSpeeds getXYSpeedsFromJoysticks(double xInput, double yInput) {


    Translation2d inputTranslation = new Translation2d(xInput, yInput);
    double magnitude = inputTranslation.getNorm();
    Rotation2d angle = 0 != xInput || 0 != yInput ? inputTranslation.getAngle() : new Rotation2d();

    double curvedMagnitude = Math.pow(magnitude, 3);


    // limit power
    double xSpeed = curvedMagnitude
        * angle.getCos()
        * drivetrain.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    double ySpeed = curvedMagnitude
        * angle.getSin()
        * drivetrain.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    return new ChassisSpeeds(xSpeed, ySpeed, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousTime.set(Timer.getFPGATimestamp());
    thetaController.reset(drivetrain.getPose().getRotation().getDegrees());
    thetaController.setGoal(side.getPose().getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Driver Input
    ChassisSpeeds driver = getXYSpeedsFromJoysticks(xInput.getAsDouble(), yInput.getAsDouble());

    // Theta Controller Velocity
    double thetaApplied = thetaController.calculate(drivetrain.getPose().getRotation().getDegrees()) * drivetrain.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    ChassisSpeeds finalSpeed = new ChassisSpeeds(driver.vxMetersPerSecond, driver.vyMetersPerSecond, thetaApplied);

    double newTime = Timer.getFPGATimestamp();
    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(previousSetpoint.get(),
        finalSpeed,
        pathConstraints,
        newTime - previousTime.get());
    swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
        newSetpoint.moduleStates(),
        newSetpoint.feedforwards().linearForces());

    previousSetpoint.set(newSetpoint);
    previousTime.set(newTime);
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
