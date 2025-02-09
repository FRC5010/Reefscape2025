// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import org.frc5010.common.motors.MotorController5010;

/** A motor that acts as a follower. */
public class FollowerMotor extends GenericFunctionalMotor {
  /**
   * Create a new follower motor.
   *
   * @param motor the motor to use as a follower
   * @param leader the leader motor
   */
  public FollowerMotor(MotorController5010 motor, MotorController5010 leader, String visualName) {
    super(motor, visualName);
    setFollow(leader);
  }

  public FollowerMotor(MotorController5010 motor, MotorController5010 leader, String visualName, boolean inversion) {
    super(motor, visualName);
    setFollow(leader, inversion);
  }
}
