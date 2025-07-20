// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.commands.calibration.PoseProviderAutoOffset;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.PoseProvider.PoseObservation;
import org.frc5010.common.drive.pose.PoseProvider.ProviderType;
import org.frc5010.common.subsystems.LEDStripSegment;
import org.frc5010.common.vision.AprilTags;
import org.frc5010.common.vision.VisionConstants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** A class to handle estimating the pose of the robot */
public class DrivePoseEstimator extends GenericSubsystem {
  /** The pose tracker */
  protected GenericPose poseTracker;
  /** The field2d object for displaying the pose */
  private final Field2d field2d;
  /** The list of AprilTag poses */
  private List<Pose2d> tagPoses = new ArrayList<>();
  /** Whether to disable the vision update command */
  private boolean disableVisionUpdateCommand = false;
  /** List of PoseProviders */
  private List<PoseProvider> poseProviders = new ArrayList<>();
  private boolean aprilTagVisible = false;
  private boolean updatingPoseAcceptor = false;

  private static double CONFIDENCE_RESET_THRESHOLD = 0.025;
  private boolean activateAcceptorUpdates = true;
  private boolean poseAcceptable = false;

  public static enum State {
    DISABLED_FIELD(ProviderType.FIELD_BASED), DISABLED_ENV(ProviderType.ENVIRONMENT_BASED),
    ENABLED_FIELD(ProviderType.FIELD_BASED), ENABLED_ENV(ProviderType.ENVIRONMENT_BASED), ALL(ProviderType.ALL),
    ODOMETRY_ONLY(ProviderType.NONE);

    public ProviderType type;

    private State(ProviderType type) {
      this.type = type;
    }
  }

  private static State state = State.DISABLED_FIELD;

  /**
   * Build a DrivePoseEstimator
   *
   * @param poseTracker the pose tracker
   */
  public DrivePoseEstimator(GenericPose poseTracker) {
    this.poseTracker = poseTracker;
    field2d = poseTracker.getField();

    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(11, 0);
    tab.addDoubleArray("Robot Pose3d", () -> getCurrentPose3dArray()).withPosition(11, 2).withSize(4, 2);

    tab.addNumber("Pose Degrees", () -> (getCurrentPose().getRotation().getDegrees()))
        .withPosition(11, 4);
    tab.add("Pose Field", field2d).withPosition(0, 0).withSize(11, 5);

    tab.addStringArray("Providers Active", () -> {
      String[] providerActive = new String[poseProviders.size()];
      for (int i = 0; i < poseProviders.size(); i++) {
        providerActive[i] = poseProviders.get(i).getClass().getSimpleName() + ": " + poseProviders.get(i).isConnected();
      }
      return providerActive;
    }).withSize(6, 2).withPosition(0, 5);
    tab.addBoolean("April Tag Visible", () -> aprilTagVisible).withPosition(6, 5);
    tab.addBoolean("Acceptor Updating", () -> updatingPoseAcceptor).withPosition(8, 5);
    tab.addString("Estimator State", () -> state.name());

    for (AprilTag at : AprilTags.aprilTagFieldLayout.getTags()) {
      if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
        field2d.getObject("Field Tag " + at.ID).setPose(at.pose.toPose2d());
        AprilTags.poseToID.put(at.pose.toPose2d(), at.ID);
        tagPoses.add(at.pose.toPose2d());
      }
    }

    new Trigger(() -> DriverStation.isDisabled()).onFalse(Commands.runOnce(() -> setState(State.ALL)));

  }

  public Function<Integer, Color8Bit> displayProviderStatuses(int length) {
    int providerLEDLength = length / poseProviders.size();
    return (Integer index) -> {
      int provider = index.intValue() / providerLEDLength;
      if (provider > poseProviders.size()) {
        return new Color8Bit(Color.kWhite);
      }
      Color color;
      color = poseProviders.get(provider).isConnected() ? Color.kGreen : Color.kRed;
      return new Color8Bit(color);
    };
  }

  public void displayOnLEDSegment(LEDStripSegment ledStrip, int length) {
    ledStrip.setLedAction(displayProviderStatuses(length));
  }

  /**
   * Register a PoseProvider to the list of pose providers.
   *
   * @param provider the PoseProvider to be registered
   */
  public void registerPoseProvider(PoseProvider provider) {
    poseProviders.add(provider);
  }

  /**
   * Set whether to disable the vision update
   *
   * @param disable whether to disable the vision update
   */
  public void setDisableVisionUpdate(boolean disable) {
    disableVisionUpdateCommand = disable;
  }

  /**
   * Get the formatted pose as a String
   *
   * @return the formatted pose
   */
  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
  }

  public Command getCalibrationCommand(GenericDrivetrain drivetrain, int cameraIndex) {
    return PoseProviderAutoOffset.createPoseProviderAutoOffset(
        this::getCurrentPose, drivetrain, new Rotation2d());
  }

  /**
   * Get the current pose on the 2D plane
   *
   * @return the current pose
   */
  public Pose2d getCurrentPose() {
    // return poseProviders.get(0).getRobotPose().get().toPose2d();
    return poseTracker.getCurrentPose();
  }

  /**
   * Get the current pose in 3d space
   *
   * @return the current pose with z = 0
   */
  public Pose3d getCurrentPose3d() {
    return new Pose3d(poseTracker.getCurrentPose());
  }

  /**
   * Get the current pose in 3d space as an array
   *
   * @return the current pose
   */
  public double[] getCurrentPose3dArray() {
    Pose3d pose = getCurrentPose3d();
    Quaternion rotation = pose.getRotation().getQuaternion();
    return new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        rotation.getW(),
        rotation.getX(),
        rotation.getY(),
        rotation.getZ()
    };
  }

  @Override
  public void periodic() {
    poseProviders.forEach(it -> it.update());
    updatePoseObservationFromProviders();
  }

  private void resetProviderPoses(Pose2d pose) {
    for (PoseProvider provider : poseProviders) {
      provider.resetPose(new Pose3d(pose));
    }
  }

  /**
   * Executes the updates of the pose estimator using the pose providers.
   *
   * <p>
   * DISABLED_FIELD - reads pose from AT camera providers and updates the Env
   * providers with low ambiguity poses from a distance
   * DISABLED_ENV - reads pose from Env providers, expecting a pose reset to be
   * provided
   * ENABLED_FIELD - reads pose from the AT camera providers and updates the Env
   * providers if the pose is high caliber and close
   * ENABLED_ENV - reads the pose from the Env providers and expects given pose to
   * be correct
   * ALL - reads and fuses poses from both Env and Field sources, resetting Env
   * pose based on update
   *
   * @return the command that updates the pose estimator
   */
  protected void updatePoseObservationFromProviders() {
    poseTracker.updateLocalMeasurements();
    boolean visionUpdated = false;
    boolean accepterUpdating = false;
    poseAcceptable = false;
    if (!disableVisionUpdateCommand) {
      for (PoseProvider provider : poseProviders.stream()
          .filter(it -> it.isConnected() && (state.type == ProviderType.ALL || it.getType() == state.type))
          .collect(Collectors.toList())) {
        List<PoseObservation> observations = provider.getObservations();
        for (PoseObservation observation : observations) {
          boolean rejectPose = (provider.getType() != ProviderType.ENVIRONMENT_BASED) &&
              observation.tagCount() == 0 // Must have at least one tag
              || (observation.tagCount() == 1
                  && observation.ambiguity() > VisionConstants.maxAmbiguity) // Cannot be high ambiguity
              || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError // Must have realistic Z coordinate

              // Must be within the field boundaries
              || observation.pose().getX() < 0.0
              || observation.pose().getX() > AprilTags.aprilTagFieldLayout.getFieldLength()
              || observation.pose().getY() < 0.0
              || observation.pose().getY() > AprilTags.aprilTagFieldLayout.getFieldWidth();

          Pose3d robotPose = observation.pose();
          if (!rejectPose) {
            visionUpdated |= true;
            poseTracker.getVisionConsumer().accept(
                robotPose.toPose2d(), observation.timestamp(), provider.getStdDeviations(observation));
          }

          // Decides if pose would be good to update
          poseAcceptable |= activateAcceptorUpdates
              && provider.getType() == ProviderType.FIELD_BASED
              && (state == State.ENABLED_FIELD || state == State.ALL)
              && observation.ambiguity() < CONFIDENCE_RESET_THRESHOLD
              && (DriverStation.isDisabled() ||
                  (!DriverStation.isDisabled()
                      && robotPose.getTranslation().getDistance(getCurrentPose3d().getTranslation()) < 0.1));
        }
      }
    }

    // Accept poses after estimation integration
    if (activateAcceptorUpdates && (poseAcceptable || state == State.DISABLED_FIELD)) {
      for (PoseProvider provider2 : poseProviders) {
        if (provider2.getType() == ProviderType.ENVIRONMENT_BASED) {
          provider2.resetPose(getCurrentPose3d());
          accepterUpdating = true;
        }
      }
    }
    SmartDashboard.putBoolean("April Tag Pose Updating", visionUpdated);
    SmartDashboard.putBoolean("Acceptor Pose Updating", accepterUpdating);
    aprilTagVisible = visionUpdated;
    updatingPoseAcceptor = accepterUpdating;
  }

  public void setState(State type) {
    state = type;
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating
   * to the software when you have manually moved your robot in a particular
   * position on the field (EX: when you place it on the field at the start of the
   * match).
   *
   * @param pose the pose to reset to
   */
  public void resetToPose(Pose2d pose) {
    poseTracker.resetToPose(pose);
    resetProviderPoses(pose);
  }

  /**
   * Get the current gyro rotation as a {@link Rotation2d}.
   *
   * @return the current gyro rotation
   */
  public Rotation2d getGyroRotation2d() {
    return poseTracker.getGyroRotation2d();
  }

  /**
   * Get the closest tag to the robot
   *
   * @return the ID of the closest tag
   */
  public int getClosestTagToRobot() {
    return AprilTags.poseToID.get(getCurrentPose().nearest(tagPoses));
  }

  /**
   * Set the target pose on the field
   *
   * @param targetPose the target pose
   * @param targetName the name of the target
   */
  public void setTargetPoseOnField(Pose2d targetPose, String targetName) {
    field2d.getObject(targetName).setPose(targetPose);
  }

  /**
   * Get the pose from the closest tag as a {@link Pose3d}
   *
   * @return the pose of the closest tag
   */
  public Pose3d getPoseFromClosestTag() {
    Pose3d targetPose = AprilTags.aprilTagFieldLayout.getTagPose(getClosestTagToRobot()).orElse(getCurrentPose3d());
    field2d.getObject("Closest Tag").setPose(targetPose.toPose2d());
    return targetPose;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
