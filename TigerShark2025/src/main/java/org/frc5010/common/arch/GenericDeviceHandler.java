package org.frc5010.common.arch;

import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public interface GenericDeviceHandler {
  public void addDevice(String name, Object device);

  public Object getDevice(String name);

  public LoggedMechanism2d getMechVisual();

  public DisplayValuesHelper getDisplayValuesHelper();
}
