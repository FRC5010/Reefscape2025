package org.frc5010.common.arch;

import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public interface GenericDeviceHandler {
  public void addDevice(String name, Object device);

  public Object getDevice(String name);

  public Mechanism2d getMechVisual();

  public DisplayValuesHelper getDisplayValuesHelper();
}
