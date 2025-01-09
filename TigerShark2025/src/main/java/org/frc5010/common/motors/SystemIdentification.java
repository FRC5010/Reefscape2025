// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.sensors.encoder.GenericEncoder;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

/** Add your docs here. */
public class SystemIdentification {
  /**
   * Tracks the voltage being applied to a motor
   */
  private static final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  /**
   * Tracks the distance travelled of a position motor
   */
  private static final MutAngle m_distance = new MutAngle(0, 0, Rotations);
  /**
   * Tracks the velocity of a positional motor
   */
  private static final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 9, RotationsPerSecond);
  /**
   * Tracks the rotations of an angular motor
   */
  private static final MutAngle m_anglePosition = new MutAngle(0, 0, Degrees);
  /**
   * Tracks the velocity of an angular motor
   */
  private static final MutAngularVelocity m_angVelocity = new MutAngularVelocity(0, 0, DegreesPerSecond);

  public static SysIdRoutine rpmSysIdRoutine(
      MotorController5010 motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(
            Volts.of(1).div(Seconds.of(1)),
            Volts.of(1),
            Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(encoder.getPosition(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
            },
            subsystemBase));
  }

  public static SysIdRoutine angleSysIdRoutine(
      MotorController5010 motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_anglePosition.mut_replace(encoder.getPosition(), Degrees))
                  .angularVelocity(
                      m_angVelocity.mut_replace(encoder.getVelocity(), DegreesPerSecond));
            },
            subsystemBase));
  }

  public static Command getSysIdQuasistatic(
      SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public static Command getSysIdQuasistaticForward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdQuasistaticBackward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kReverse);
  }

  public static Command getSysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public static Command getSysIdDynamicForward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdDynamicBackward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kReverse);
  }

  public static Command getSysIdFullCommand(
      SysIdRoutine routine, double quasistaticTimeout, double dynamicTimeout, double delay) {
    return getSysIdQuasistaticForward(routine)
        .withTimeout(quasistaticTimeout)
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdQuasistaticBackward(routine).withTimeout(quasistaticTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicForward(routine).withTimeout(dynamicTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicBackward(routine).withTimeout(dynamicTimeout));
  }
}
