package org.frc5010.common.config.json.devices;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.sensors.gyro.NavXGyro;
import org.frc5010.common.sensors.gyro.PigeonGyro;
import org.frc5010.common.sensors.gyro.YagslGyro;

import com.studica.frc.AHRS.NavXComType;

/** Instatiates a Gyro sensor based on the specified type */
public class GyroSettingsConfigurationJson implements DeviceConfiguration {
  /** The type of gyro sensor */
  public String type;
  /** The ID of the gyro sensor, if needed */
  public int id;

/**
 * Configures and returns an appropriate gyro sensor based on the specified type.
 *
 * @param mechanismSimulation The LoggedMechanism2d instance for visualization, if needed.
 * @return An instance of a gyro sensor (NavXGyro or PigeonGyro) based on the type, or null if the type is unrecognized.
 */
  @Override
  public Object configure(GenericSubsystem deviceHandler) {
    switch (type) {
      case "navx":
        return new NavXGyro(NavXComType.kMXP_SPI);
      case "pigeon2":
        return new PigeonGyro(id);
      case "yagsl":
        return new YagslGyro();
      default:
        return null;
    }
  }
}
