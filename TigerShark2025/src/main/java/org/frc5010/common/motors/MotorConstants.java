// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Function;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class MotorConstants {
  public static enum Motor {
    Neo(Amps.of(40), RPM.of(5676), (Integer numberOfMotors) -> DCMotor.getNEO(numberOfMotors), 42), 
    Neo550(Amps.of(20), RPM.of(11000),  (Integer numberOfMotors) -> DCMotor.getNeo550(numberOfMotors), 42), 
    KrakenX60(Amps.of(40), RPM.of(6000),  (Integer numberOfMotors) -> DCMotor.getKrakenX60(numberOfMotors), 1023)
    ;

    public Current currentLimit = Amps.of(20);
    public AngularVelocity maxRpm;
    public Function<Integer, DCMotor> motorSim;
    public double ticsPerRotation;
    
    private Motor(Current currentLimit, AngularVelocity rpm, Function<Integer, DCMotor> motorSim, double ticsPerRotation) {
      this.currentLimit = currentLimit;
      maxRpm = rpm;
      this.motorSim = motorSim;
    }
    public DCMotor getMotorSimulationType() {
      return getMotorSimulationType(1);
    }

    public DCMotor getMotorSimulationType(int numberOfMotors) {
      return motorSim.apply(numberOfMotors);
    }
  }
}
