// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class MotorConstants {
  public static enum Motor {
    Neo(Amps.of(40), RPM.of(5676), DCMotor.getNEO(1)), 
    Neo550(Amps.of(20), RPM.of(11000), DCMotor.getNeo550(1)), 
    KrakenX60(Amps.of(20), RPM.of(6000), DCMotor.getKrakenX60(1))
    ;

    public Current currentLimit = Amps.of(20);
    public AngularVelocity maxRpm;
    public DCMotor motorSim;
    
    private Motor(Current currentLimit, AngularVelocity rpm, DCMotor motorSim) {
      this.currentLimit = currentLimit;
      maxRpm = rpm;
      this.motorSim = motorSim;
    }
    private Motor currentLimit(Current limit) {
      currentLimit = limit;
      return this;
    }
  }
}
