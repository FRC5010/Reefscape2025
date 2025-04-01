// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.PositionControlMotor;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbSubsystem extends GenericSubsystem {
  PositionControlMotor climbMotor;
  GenericEncoder climbMotorEncoder;
  private final double GEAR_RATIO = 60;
  private final Distance SPOOL_RADIUS = Inches.of(0.75 / 2.0);
  private final Distance PIVOT_TO_ATTACHMENT = Inches.of(10);
  private final Translation2d PIVOT_LOCATION = new Translation2d(Inches.of(20), Inches.of(20));

  private final double CLIMB_LIMIT_PEAK = 2.70;
  private final double CLIMB_LIMIT_MIN = -4.4;

  public static enum ClimbPosition {
    RETRACTED(Rotations.of(-10.0)),
    EXTENDED(Rotations.of(10.0)); // TODO: Fix number of rotations

    private final Angle position;

    ClimbPosition(Angle position) {
      this.position = position;
    }

    public Angle getPosition() {
      return position;
    }
  }


  private Distance rotationsToStringLength(double rotations) {
    return SPOOL_RADIUS.times(2).times(Math.PI).times(rotations);
  }

  private Angle spoolRotationsToDegrees(double rotations) {
    double stringLength = rotationsToStringLength(rotations).in(Meters);
    double SideB = PIVOT_LOCATION.getNorm();
    double ClimbSide = PIVOT_TO_ATTACHMENT.in(Meters);

    double climbAngle = Math.acos((Math.pow(stringLength, 2) - Math.pow(SideB, 2) - Math.pow(ClimbSide, 2)) / ( -2 * SideB * ClimbSide));
    return Radians.of(climbAngle);
  }

  public Angle getClimbAngle() {
    return spoolRotationsToDegrees(climbMotorEncoder.getPosition());
  }

  public boolean isAtMax() {
    return climbMotor.getMotorEncoder().getPosition() > CLIMB_LIMIT_PEAK;
  }

  public boolean isAtMin() {
    return climbMotor.getMotorEncoder().getPosition() < CLIMB_LIMIT_MIN;
  }

  public void runMotor(double speed) {
    if (speed > 0 && isAtMax() || speed < 0 && isAtMin()) {
      climbMotor.set(0);
      return;
    }
    climbMotor.set(speed);

  }

  public ClimbSubsystem() {
    climbMotor = new PositionControlMotor(MotorFactory.TalonFX(13, Motor.KrakenX60), "Climb Motor", new DisplayValuesHelper("Motors", "Climb Motor"));
    climbMotorEncoder = climbMotor.getMotorEncoder();
    climbMotorEncoder.setPositionConversion((1 / GEAR_RATIO));
    climbMotorEncoder.setPosition(0.0);
  }

  public Command retractClimb() {
    return Commands.run(() -> runMotor(-1.0), this).until(() -> getRotation() <= ClimbPosition.RETRACTED.getPosition().in(Rotations));
  }

  public Command extendClimb() {
    return Commands.run(() -> runMotor(1.0), this).until(() -> getRotation() >= ClimbPosition.EXTENDED.getPosition().in(Rotations));
  }

  public Command runClimb(DoubleSupplier speed) {
    return Commands.run(() -> runMotor(speed.getAsDouble()), this);
  }

  public double getRotation() {
    return climbMotorEncoder.getPosition();
  }

  public boolean isClimbing() {
    return Math.abs(climbMotorEncoder.getVelocity()) > 0.01;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Motor Position", getRotation());
  }
}
