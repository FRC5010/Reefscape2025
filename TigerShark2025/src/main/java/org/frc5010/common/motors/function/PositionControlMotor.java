// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.telemetry.DisplayValuesHelper;

/** Add your docs here. */
public class PositionControlMotor extends GenericControlledMotor {
  public PositionControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
    super(motor, visualName, tab);
  }

  @Override
  public double getEncoderFeedback() {
    return 0;
  }
  
  @Override
  public Command getSysIdCommand(SubsystemBase subsystemBase) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSysIdCommand'");
  }
}
