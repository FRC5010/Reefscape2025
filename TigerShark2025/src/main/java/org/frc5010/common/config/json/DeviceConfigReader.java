package org.frc5010.common.config.json;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import org.frc5010.common.arch.GenericDeviceHandler;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.MotorConstants.Motor;

/** A class for reading device configurations from JSON files. */
public class DeviceConfigReader {
  /**
   * Retrieves a motor controller based on the specified type and ID.
   *
   * @param type The type of motor, e.g., "neo" or "kraken".
   * @param id   The unique identifier for the motor.
   * @return A MotorController5010 instance for the specified type and ID, or null
   *         if the type is not recognized.
   */
  public static MotorController5010 getMotor(String controller, String type, int id) {
    MotorController5010 motor;
    switch (controller) {
      case "spark":
        motor = MotorFactory.Spark(id, Motor.valueOf(type));
        break;
      case "talonfx":
        motor = MotorFactory.TalonFX(id, Motor.valueOf(type));
        break;
      case "thrifty":
        motor = MotorFactory.Thrifty(id, Motor.valueOf(type));
        break;
      default:
        return null;
    }
    return motor;
  }

  /**
   * Reads a device configuration from the given file and adds it to the system.
   *
   * @param system     the system to add the device to
   * @param deviceFile the file containing the device configuration
   * @param key        the key of the device to read (e.g., "gyro",
   *                   "percent_motor", etc.)
   * @throws StreamReadException if the file cannot be read
   * @throws DatabindException   if the file cannot be parsed
   * @throws IOException         if there is an error reading the file
   */
  public static void readDeviceConfig(GenericDeviceHandler system, File deviceFile, String key)
      throws StreamReadException, DatabindException, IOException {
    switch (key) {
      case "gyro":
        GyroSettingsConfigurationJson gyroConfig = new ObjectMapper().readValue(deviceFile,
            GyroSettingsConfigurationJson.class);
        system.addDevice(ConfigConstants.GYRO, gyroConfig.configure(system));
        break;
      case "percent_motor":
        PercentMotorConfigurationJson percentMotorConfig = new ObjectMapper().readValue(deviceFile,
            PercentMotorConfigurationJson.class);
        system.addDevice(
            percentMotorConfig.name, percentMotorConfig.configure(system));
        break;
      case "velocity_motor":
        VelocityMotorConfigurationJson motorConfigurationJson = new ObjectMapper().readValue(deviceFile,
            VelocityMotorConfigurationJson.class);
        system.addDevice(
            motorConfigurationJson.name, motorConfigurationJson.configure(system));
        break;
      default:
        break;
    }
  }
}
