package org.frc5010.common.config;

import org.frc5010.common.arch.GenericSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface DeviceConfiguration {
  public default Object configure(SubsystemBase deviceHandler) { return null; }
  public default Object configure(GenericSubsystem deviceHandler) { return configure((SubsystemBase)deviceHandler); }
}
