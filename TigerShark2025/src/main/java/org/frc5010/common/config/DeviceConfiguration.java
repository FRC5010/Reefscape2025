package org.frc5010.common.config;

import org.frc5010.common.arch.GenericDeviceHandler;

public interface DeviceConfiguration {
  public Object configure(GenericDeviceHandler deviceHandler);
}
