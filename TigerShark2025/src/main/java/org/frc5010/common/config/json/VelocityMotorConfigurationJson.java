package org.frc5010.common.config.json;

import org.frc5010.common.arch.GenericDeviceHandler;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.function.VelocityControlMotor;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Configures a VelocityControlMotor with the given parameters and visualizes it. */
public class VelocityMotorConfigurationJson implements DeviceConfiguration {
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
  /** The x position */
  public double x;
  /** The y position */
  public double y = 0.0;
  /** The z position */
  public double z;
  /** The kS value */
  public double kS = 0.0;
  /** The kV value */
  public double kV = 0.0;
  /** The kA value */
  public double kA = 0.0;
  /** The kP value */
  public double kP = 0.0;
  /** The kI value */
  public double kI = 0.0;
  /** The kD value */
  public double kD = 0.0;
  /** The iZone value */
  public double iZone = 0.0;

  /**
   * Configures a VelocityControlMotor with the given parameters and visualizes it.
   *
   * @param mechanismSimulation The Mechanism2d instance for visualization.
   * @return A configured VelocityControlMotor object.
   */
  @Override
  public Object configure(GenericDeviceHandler deviceHandler) {
    VelocityControlMotor motor =
        new VelocityControlMotor(DeviceConfigReader.getMotor(type, id), name, deviceHandler.getDisplayValuesHelper())
            .setupSimulatedMotor(gearing, momentOfInertiaKgMSq)
            .setVisualizer(deviceHandler.getMechVisual(), new Pose3d(x, y, z, new Rotation3d()));
    if (kP != 0.0 || kI != 0.0 || kD != 0.0) {
      motor.setValues(new GenericPID(kP, kI, kD));
    }
    if (kS != 0.0 || kV != 0.0 || kA != 0.0) {
      motor.setMotorFeedFwd(new MotorFeedFwdConstants(kS, kV, kA));
    }
    if (iZone != 0.0) {
      motor.setIZone(iZone);
    }
    return motor;
  }
}
