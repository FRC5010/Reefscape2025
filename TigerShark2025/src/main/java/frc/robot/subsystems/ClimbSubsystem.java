// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.PositionControlMotor;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbSubsystem extends GenericSubsystem {
  PositionControlMotor climbMotor;
  GenericEncoder climbMotorEncoder;

  public static enum ClimbPosition {
    RETRACTED(Rotations.of(0.0)),
    EXTENDED(Rotations.of(10.0)); // TODO: Fix number of rotations

    private final Angle position;

    ClimbPosition(Angle position) {
      this.position = position;
    }

    public Angle getPosition() {
      return position;
    }
  }

  public ClimbSubsystem() {
    climbMotor = new PositionControlMotor(MotorFactory.TalonFX(13, Motor.KrakenX60), "Climb Motor", new DisplayValuesHelper("Motors", "Climb Motor"));
    climbMotorEncoder = climbMotor.getMotorEncoder();
  }

  public void retractClimb() {
    Commands.runOnce(() -> climbMotor.set(-0.25)).until(() -> getRotation() <= ClimbPosition.RETRACTED.getPosition().in(Rotations));
  }

  public void extendClimb() {
    Commands.runOnce(() -> climbMotor.set(0.25)).until(() -> getRotation() >= ClimbPosition.EXTENDED.getPosition().in(Rotations));
  }

  public double getRotation() {
    return climbMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
