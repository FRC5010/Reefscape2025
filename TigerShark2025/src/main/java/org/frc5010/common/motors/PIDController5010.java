// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;

public interface PIDController5010 {
  public static enum PIDControlType {
    NONE,
    POSITION,
    VELOCITY,
    VOLTAGE,
    CURRENT,
    DUTY_CYCLE,
    PROFILED_POSITION,
    PROFILED_VELOCITY
  }

  public void setTolerance(double tolerance);

  public double getTolerance();

  public void setValues(GenericPID pid);

  public void setMotorFeedFwd(MotorFeedFwdConstants motorConstants);

  public void setP(double p);

  public void setI(double i);

  public void setD(double d);

  public void setF(double f);

  public void setIZone(double iZone);

  public void setOutputRange(double min, double max);

  public void setReference(double reference);

  public void setReference(double reference, PIDControlType controlType, double feedforward);

  public double getReferenceVelocity();

  public void setControlType(PIDControlType controlType);

  public void setProfiledMaxVelocity(double maxVelocity);

  public void setProfiledMaxAcceleration(double maxAcceleration);

  public GenericPID getValues();

  public double getP();

  public double getI();

  public double getD();

  public double getF();

  public double getIZone();

  public double getReference();

  public PIDControlType getControlType();

  public boolean isAtTarget();

  public void resetController(double position, double velocity);

  public double calculateControlEffort(double current);

  public void configureAbsoluteControl(double offset, boolean inverted, double min, double max);
}
