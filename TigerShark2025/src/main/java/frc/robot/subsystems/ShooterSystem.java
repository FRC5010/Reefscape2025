// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.Constants;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.sensors.Beambreak;
import org.frc5010.common.sensors.LaserCAN;
import org.frc5010.common.sensors.SparkLimit;
import org.frc5010.common.sensors.ValueSwitch;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;

public class ShooterSystem extends GenericSubsystem {
  Beambreak alignmentBeambreak;
  LaserCAN entryBeambreak;

  protected ValueSwitch currentSwitch;
  protected VelocityControlMotor shooterLeft;
  protected VelocityControlMotor shooterRight;
  protected SparkLimit beambreakLimit;
  private Trigger entryBroken, alignmentBroken;
  private Trigger isEmpty, isEntryActive, isCoralFullyCaptured, isAligned, isStopped;
  private CoralState coralState;
  private DisplayBoolean entryBeamBreakDisplay, alignmentBeamBreakDisplay;
  private double captureEncoderCount = 0.0;
  private int stoppedCount = 0;
  private BooleanSupplier isInLoadZone = () -> true;
  private IntakeSimulation intakeSimulation;
  private SwerveDriveSimulation driveSimulation;
  private Supplier<Distance> elevatorHeight = () -> Meters.zero();
  
  public enum CoralState {
    EMPTY,
    ENTRY,
    FULLY_CAPTURED,
    ALIGNED
  }

  public static class Config {
    private final Current INTAKE_CURRENT_THRESHOLD = Amps.of(10);
    int alignmentBeambreakCanID = 8;
    int entryBeambreakCanID = 0;
    int shooterLeftCanID = 11;
    int shooterRightCanID = 12;
    boolean entryBeambreakDisplay = false;
    boolean alignmentBeambreakDisplay = false;
    double currentSwitchTriggerThreshold = 1.0;
    boolean shooterLeftInverted = false;
    boolean shooterRightInverted = true;
    double shooterLeftGearing = 1.0;
    double shooterRightGearing = 1.0;
    double shooterLeftJKGMetersSquared = 10.0;
    double shooterRightJKGMetersSquared = 10.0;
    Distance shooterLeftX = Inches.of(7.15);
    Distance shooterLeftY = Inches.of(2.875);
    Distance shooterLeftZ = Inches.of(16.25);
    Distance shooterRightX = Inches.of(7.15);
    Distance shooterRightY = Inches.of(2.875);
    Distance shooterRightZ = Inches.of(16.25);
    double runMotorsSpeedMultiplier = 0.5;
  }

  private Config config = new Config();

  /** Creates a new Shooter. */
  public ShooterSystem(Mechanism2d mechanismSimulation, Config config) {
    if (config != null)
      this.config = config;

    setupMotors(mechanismSimulation);

    beambreakLimit = new SparkLimit((SparkMax) shooterRight.getMotor());
    alignmentBeambreak = new Beambreak(() -> beambreakLimit.getForwardLimit());
    entryBeambreak = new LaserCAN(config.entryBeambreakCanID);

    entryBeamBreakDisplay = displayValues.makeDisplayBoolean("Entry Beam Break");
    alignmentBeamBreakDisplay = displayValues.makeDisplayBoolean("Alignment Beam Break");

    entryBeamBreakDisplay.setValue(config.entryBeambreakDisplay);
    alignmentBeamBreakDisplay.setValue(config.alignmentBeambreakDisplay);

    entryBroken = new Trigger(() -> entryBeambreak.getDistance() == 0);
    alignmentBroken = new Trigger(alignmentBeambreak.isBrokenSupplier());

    coralState = alignmentBeambreak.isBroken() ? CoralState.FULLY_CAPTURED : CoralState.EMPTY;

    isEmpty = new Trigger(() -> coralState == CoralState.EMPTY);
    isEntryActive = new Trigger(() -> coralState == CoralState.ENTRY);
    isCoralFullyCaptured = new Trigger(() -> coralState == CoralState.FULLY_CAPTURED);
    isAligned = new Trigger(() -> coralState == CoralState.ALIGNED);
    isStopped = new Trigger(() -> getStoppedCount() > 10);

    currentSwitch = new ValueSwitch(config.INTAKE_CURRENT_THRESHOLD.in(Amps), shooterRight::getOutputCurrent,
        config.currentSwitchTriggerThreshold);

    captureEncoderCount = shooterLeft.getMotorEncoder().getPosition();

    // isStopped.and(currentSwitch.getTrigger()).and(isInLoadZone)
    // .onTrue(setCoralState(CoralState.FULLY_CAPTURED)); // TODO: Calibrate
    // threshold

    setupStateMachine();
    // isEntryActive.whileTrue(captureCoral());
    // isCoralFullyCaptured.whileTrue(alignCoral());

    isCoralFullyCaptured
        .onTrue(Commands.runOnce(() -> captureEncoderCount = shooterLeft.getMotorEncoder().getPosition()));

    // isCoralFullyCaptured.and(() -> Math.abs(captureEncoderCount -
    // shooterLeft.getMotorEncoder().getPosition()) > 0.75)
    // .onTrue(setCoralState(CoralState.ALIGNED));

    if (RobotBase.isSimulation()) {
      driveSimulation = YAGSLSwerveDrivetrain.getSwerveDrive().getMapleSimDrive().get();

      intakeSimulation = IntakeSimulation.InTheFrameIntake(Constants.Simulation.gamePieceA,
          driveSimulation, Inches.of(19.113), IntakeSide.BACK, 1);
      intakeSimulation.addGamePieceToIntake(); // Pre-load
    }
  }

  public void setElevatorHeightSupplier(Supplier<Distance> supplier) {
    this.elevatorHeight = supplier;
  }

  private void setupMotors(Mechanism2d mechanismSimulation) {
    shooterLeft = new VelocityControlMotor(MotorFactory.Spark(config.shooterLeftCanID, Motor.Neo), "shooterLeft",
        displayValues);
    shooterRight = new VelocityControlMotor(MotorFactory.Spark(config.shooterRightCanID, Motor.Neo), "shooterRight",
        displayValues);

    shooterLeft.setCurrentLimit(Amps.of(100));
    shooterRight.setCurrentLimit(Amps.of(100));
    shooterLeft.invert(config.shooterLeftInverted);
    shooterRight.invert(config.shooterRightInverted);

    shooterLeft.setupSimulatedMotor(config.shooterLeftGearing, config.shooterLeftJKGMetersSquared);
    shooterRight.setupSimulatedMotor(config.shooterRightGearing, config.shooterRightJKGMetersSquared);
    shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(
        new Translation3d(config.shooterLeftX.in(Meters), config.shooterLeftY.in(Meters),
            config.shooterLeftZ.in(Meters)),
        new Rotation3d()));
    shooterRight.setVisualizer(mechanismSimulation,
        new Pose3d(new Translation3d(config.shooterRightX.in(Meters), config.shooterRightY.in(Meters),
            config.shooterRightZ.in(Meters)), new Rotation3d()));
  }

  private void setupStateMachine() {
    Trigger coralOutOfShooter = entryBroken.negate().and(alignmentBroken.negate());

    coralOutOfShooter.onTrue(setCoralState(CoralState.EMPTY));

    // Empty
    isEmpty().and(entryBroken).onTrue(setCoralState(CoralState.ENTRY));

    // Entry
    (isEntryActive.or(isEmpty())).and(alignmentBroken).and(entryBroken.negate())
        .onTrue(setCoralState(CoralState.FULLY_CAPTURED));
    // Fully Captured

    alignmentBroken.and(() -> DriverStation.isFMSAttached()).and(() -> RobotState.isDisabled())
        .whileTrue(setCoralState(CoralState.FULLY_CAPTURED));

  }

  public Trigger coralCapturedOrAligned() {
    return isFullyCaptured().or(isAligned());
  }

  public void shooterLeftSpeed(double speed) {
    SmartDashboard.putNumber("Shooter Left Set", speed);
    shooterLeft.set(speed);
  }

  public Trigger obtainedGamePieceToScore = new Trigger(() -> {
    return RobotBase.isSimulation()
        ? intakeSimulation.getGamePiecesAmount() == 1 && intakeSimulation.obtainGamePieceFromIntake()
        : false;
  });

  public Trigger gamePieceIsInsideIntake = new Trigger(() -> {
    return RobotBase.isSimulation() ? intakeSimulation.getGamePiecesAmount() > 0 : false;
  });

  public void shooterRightSpeed(double speed) {
    SmartDashboard.putNumber("Shooter Right Set", speed);
    shooterRight.set(-speed);
  }

  public void setLoadZoneSupplier(BooleanSupplier supplier) {
    isInLoadZone = supplier;
  }

  public Command runMotors(DoubleSupplier speed) {
    return Commands.run(() -> {
      shooterLeftSpeed(speed.getAsDouble());
      shooterRightSpeed(speed.getAsDouble());
    }, this);
  }

  public Command runMotors(DoubleSupplier speedLeft, DoubleSupplier speedRight) {
    return Commands.run(() -> {
      shooterLeftSpeed(speedLeft.getAsDouble());
      shooterRightSpeed(speedRight.getAsDouble());
    }, this);
  }

  public Command shootL1() {
    return Commands.run(() -> {
      shooterLeftSpeed(0.3);
      shooterRightSpeed(0.1);
    }, this);
  }

  public Command getShootCommand(ScoringLevel level) {
    if (RobotBase.isSimulation() && intakeSimulation != null) {
      if (intakeSimulation.getGamePiecesAmount() > 0) {
        Distance height = elevatorHeight.get();
        if (intakeSimulation.obtainGamePieceFromIntake()
          // Add this to prevent shooting from low.
          //&& height.compareTo(frc.robot.Constants.MIN_SHOOTER_HEIGHT.plus(Meters.of(0.05))) >= 0
          ) {
          SimulatedArena.getInstance()
              .addGamePieceProjectile(new ReefscapeCoralOnFly(
                  // Obtain robot position from drive simulation
                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                  // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                  new Translation2d(0.125, 0),
                  // Obtain robot speed from drive simulation
                  driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  // Obtain robot facing from drive simulation
                  driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                  // The height at which the coral is ejected
                  height.plus(frc.robot.Constants.MIN_SHOOTER_HEIGHT),
                  // The initial speed of the coral
                  MetersPerSecond.of(2),
                  // The coral is ejected at a 45-degree slope
                  Degrees.of(-45)));
        }
      }
    }
    switch (level) {
      case L1:
        return shootL1();
      case L2:
        return runMotors(() -> 0.3);
      case L3:
        return runMotors(() -> 0.3);
      case L4:
        return runMotors(() -> 0.4);
      default:
        return runMotors(() -> 0.4);
    }
  }

  public Command captureCoral() {
    return runMotors(() -> 0.05);
  }

  public Command alignCoral() {
    return runMotors(() -> 0.1).until(isAligned);
  }

  public Command stopMotors() {
    return Commands.runOnce(() -> {
      shooterLeftSpeed(0);
      shooterRightSpeed(0);
    });
  }

  public void setMotorSpeedZero() {
    shooterLeftSpeed(0.0);
    shooterRightSpeed(0.0);
  }

  public Command setCoralState(CoralState state) {
    return Commands.runOnce(() -> coralState = state).ignoringDisable(true);
  }

  public CoralState getCoralState() {
    return coralState;
  }

  public Command intakeCoral() {
    return runMotors(() -> 0.7)
        .beforeStarting(() -> {
          if (RobotBase.isSimulation()) {
            intakeSimulation.startIntake();
          }
        })
        .until(isFullyCaptured()).finallyDo(() -> {
          if (RobotBase.isSimulation()) {
            intakeSimulation.stopIntake();
          }
          shooterLeftSpeed(0);
          shooterRightSpeed(0);
        });
  }

  public Trigger isEmpty() {
    return isEmpty;
  }

  public Trigger isAligned() {
    return isAligned;
  }

  public Trigger isFullyCaptured() {
    return isCoralFullyCaptured;
  }

  public Trigger coralHasEntered() {
    return isEntryActive;
  }

  public double getAverageMotorSpeed() {
    return (Math.abs(shooterLeft.getMotorEncoder().getVelocity())
        + Math.abs(shooterRight.getMotorEncoder().getVelocity())) / 2;
  }

  public int getStoppedCount() {
    if (getAverageMotorSpeed() < 20) { // TODO: Fix Threshold
      stoppedCount++;
    } else {
      stoppedCount = 0;
    }
    return stoppedCount;
  }

  public Trigger isStopped() {
    return isStopped;
  }

  public Trigger entrySensorIsBroken() {
    return entryBroken;
  }

  @Override
  public void periodic() {
    shooterLeft.periodicUpdate();
    shooterRight.periodicUpdate();
    entryBeamBreakDisplay.setValue(entryBroken.getAsBoolean());
    alignmentBeamBreakDisplay.setValue(alignmentBroken.getAsBoolean());

    SmartDashboard.putString("Coral State", coralState.name());
    SmartDashboard.putNumber("Shooter Left Current", shooterLeft.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Right Current", shooterRight.getOutputCurrent());
    SmartDashboard.putBoolean("Spark Limit Forward", beambreakLimit.getForwardLimit());
    SmartDashboard.putNumber("Shooter Encoder Difference",
        Math.abs(captureEncoderCount - shooterLeft.getMotorEncoder().getPosition()));
    SmartDashboard.putNumber("Shooter Left Speed", shooterLeft.getMotorEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Right Speed", shooterRight.getMotorEncoder().getVelocity());

  }

  @Override
  public void simulationPeriodic() {
    shooterLeft.simulationUpdate();
    shooterRight.simulationUpdate();
  }

  public Command getSysIdCommand() {
    return shooterLeft.getSysIdCommand(this).andThen(shooterRight.getSysIdCommand(this));
  }
}
