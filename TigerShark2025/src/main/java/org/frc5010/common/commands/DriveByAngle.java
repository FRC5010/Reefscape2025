package org.frc5010.common.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.mechanisms.DriveConstantsDef;

/** Drive the robot where the angle is given */
public class DriveByAngle extends Command {
  // TODO: Understand code
  /** The subsystem used by this command. */
  private final GenericDrivetrain drivetrainSubsystem;

  /** The translation supplier in the X direction. */
  private final DoubleSupplier m_translationXSupplier;
  /** The translation supplier in the Y direction. */
  private final DoubleSupplier m_translationYSupplier;
  /** The rotation supplier in the X direction */
  private final DoubleSupplier m_rotationXSupplier;
  /** The rotation supplier in the Y direction */
  private final DoubleSupplier m_rotationYSupplier;
  /** Whether to use field oriented drive */
  private Supplier<Boolean> fieldOrientedDrive;

  /** The joystick simulation */
  private MechanismRoot2d joystick;
  /** The X axis simulation */
  private MechanismLigament2d xAxis;
  /** The Y axis simulation */
  private MechanismLigament2d yAxis;
  /** The heading axis simulation */
  private MechanismLigament2d heading;
  /** The max chassis velocity */
  private double maxChassisVelocity = Preferences.getDouble(DriveConstantsDef.MAX_CHASSIS_VELOCITY, 15);
  /** The max chassis rotation rate */
  private double maxChassisRotation = Preferences.getDouble(DriveConstantsDef.MAX_CHASSIS_ROTATION, 1.5);

  /**
   * Creates a new DriveByAngle command.
   *
   * @param drivetrainSubsystem  the subsystem used by this command
   * @param translationXSupplier the translation supplier in the X direction
   * @param translationYSupplier the translation supplier in the Y direction
   * @param rotationXSupplier    the rotation supplier in the X direction
   * @param rotationYSupplier    the rotation supplier in the Y direction
   * @param fieldOrientedDrive   whether to use field oriented drive
   */
  public DriveByAngle(
      GenericDrivetrain drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationXSupplier,
      DoubleSupplier rotationYSupplier,
      Supplier<Boolean> fieldOrientedDrive) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationXSupplier = rotationXSupplier;
    this.m_rotationYSupplier = rotationYSupplier;
    this.fieldOrientedDrive = fieldOrientedDrive;

    joystick = drivetrainSubsystem.getMechVisual().getRoot("joystick", 30, 30);
    xAxis = new MechanismLigament2d("xAxis", 1, 90, 6, new Color8Bit(Color.kDarkRed));
    yAxis = new MechanismLigament2d("yAxis", 1, 180, 6, new Color8Bit(Color.kDarkSalmon));
    heading = new MechanismLigament2d("heading", 20, 0, 6, new Color8Bit(Color.kDeepPink));
    joystick.append(xAxis);
    joystick.append(yAxis);
    joystick.append(heading);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double x = m_translationXSupplier.getAsDouble();
    double y = m_translationYSupplier.getAsDouble();
    double r = Math.max(
        Math.min(
            Math.atan2(m_rotationXSupplier.getAsDouble(), m_rotationYSupplier.getAsDouble())
                - drivetrainSubsystem.getHeading().getRadians(),
            1),
        -1);

    xAxis.setLength(x * 30);
    yAxis.setLength(y * 30);
    heading.setAngle(180 * r);

    if (fieldOrientedDrive.get()) {
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          x * maxChassisVelocity,
          y * maxChassisVelocity,
          r * maxChassisRotation, drivetrainSubsystem.getHeading());
      drivetrainSubsystem.drive(chassisSpeeds,
          null);
    } else {
      drivetrainSubsystem.drive(
          new ChassisSpeeds(
              x * maxChassisVelocity, y * maxChassisVelocity, r * maxChassisRotation),
          null);
    }
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented
    // movement
    // drivetrainSubsystem.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r,
    // drivetrainSubsystem.getHeading())
    // );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0), null);
  }
}
