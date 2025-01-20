// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class MotorConstants {
  public static class CurrentLimits {
    public static final Current Neo = Amps.of(40);
    public static final Current Neo550 = Amps.of(20);
    public static final Current KrakenX60 = Amps.of(80);
  }
  
  public static class MaxRpms {
    public static final AngularVelocity Neo = RPM.of(5676);
    public static final AngularVelocity Neo550 = RPM.of(11000);
    public static final AngularVelocity KralenX60 = RPM.of(6000);
  }
}
