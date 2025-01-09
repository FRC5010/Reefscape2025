package org.frc5010.common.config.json;

import org.frc5010.common.arch.GenericDeviceHandler;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.motors.function.PercentControlMotor;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Configures a PercentControlMotor with the given parameters and visualizes it. */
public class PercentMotorConfigurationJson implements DeviceConfiguration {
  /** The name of the motor */
  public String name;
  /** The type of motor */
  public String type;
  /** The ID of the motor */
  public int id;
  /** The gearing of the motor */
  public double gearing;
  /** The moment of inertia of the motor */
  public double momentOfInertiaKgMSq;
  /** The x position of the motor */
  public double x;
  /** The y position of the motor */
  public double y;
  /** The z position of the motor */
  public double z;

/**
 * Configures a PercentControlMotor with the given parameters and visualizes it.
 *
 * @param mechanismSimulation The Mechanism2d instance for visualization.
 * @return A configured PercentControlMotor object.
 */
  @Override
  public Object configure(GenericDeviceHandler deviceHandler) {
    return new PercentControlMotor(DeviceConfigReader.getMotor(type, id), name)
        .setupSimulatedMotor(gearing, momentOfInertiaKgMSq)
        .setVisualizer(deviceHandler.getMechVisual(), new Pose3d(x, y, z, new Rotation3d()));
  }
}
