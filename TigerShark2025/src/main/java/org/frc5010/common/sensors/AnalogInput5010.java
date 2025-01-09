// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import org.frc5010.common.sensors.encoder.GenericEncoder;

/** A class for supporting an analog input on the roboRIO */
public class AnalogInput5010 implements GenericEncoder {

  private AnalogInput analogInput;
  private boolean inverted;

  /**
   * AnalogInput5010: Creates a new AnalogInput5010
   *
   * @param port - The port of the analog input
   */
  public AnalogInput5010(int port) {
    this.analogInput = new AnalogInput(port);
    this.inverted = false;
  }

  @Override
  public double getPosition() {
    return (inverted ? -1.0 : 1.0)
        * ((analogInput.getAverageVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2)
            - Math.PI);
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void reset() {}

  @Override
  public void setPositionConversion(double conversion) {}

  @Override
  public void setVelocityConversion(double conversion) {}

  @Override
  public void setPosition(double position) {}

  @Override
  public void setRate(double rate) {}

  @Override
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }
}
