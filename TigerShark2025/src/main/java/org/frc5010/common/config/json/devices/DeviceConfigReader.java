package org.frc5010.common.config.json.devices;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.config.json.devices.ReflectionsManager.SparkBaseType;
import org.frc5010.common.config.json.devices.ReflectionsManager.VENDOR;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorFactory;

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
  public static void readDeviceConfig(GenericSubsystem system, File deviceFile, String key)
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
      case "yams_elevator":
        YamsElevatorConfigurationJson yamsElevatorConfigurationJson = new ObjectMapper().readValue(deviceFile,
            YamsElevatorConfigurationJson.class);
        system.addDevice(
            yamsElevatorConfigurationJson.motorSetup.name, yamsElevatorConfigurationJson.configure(system));
        break;
      case "yams_arm":
        YamsArmConfigurationJson yamsArmConfigurationJson = new ObjectMapper().readValue(deviceFile,
            YamsArmConfigurationJson.class);
        system.addDevice(
            yamsArmConfigurationJson.motorSetup.name, yamsArmConfigurationJson.configure(system));
        break;
      case "yams_turret":
        YamsPivotConfigurationJson yamsTurretConfigurationJson = new ObjectMapper().readValue(deviceFile,
            YamsPivotConfigurationJson.class);
        system.addDevice(
            yamsTurretConfigurationJson.motorSetup.name, yamsTurretConfigurationJson.configure(system));
        break;
      default:
        break;
    }
  }

  /**
   * Get a {@link SmartMotorController} wrapper from the provided motor controller
   * object and simulate
   * it with the provided motor simulator.
   *
   * @param controller Motor controller type. Supported types are "spark",
   *                   "sparkmax", "talonfx",
   *                   "talonfxs", "nova", "thrifty", "thrifty_nova", and
   *                   "thriftynova".
   * @param type       Motor type. Supported types are "neo", "neo550",
   *                   "krakenx60", and "krakenx60foc".
   * @param id         Motor ID.
   * @param config     {@link SmartMotorControllerConfig} for the motor
   *                   controller.
   * @return {@link SmartMotorController} wrapper for the motor controller.
   */
  public static Optional<SmartMotorController> getSmartMotor(String controller, String type, int id,
      SmartMotorControllerConfig config) {
    Optional<SmartMotorController> motor = Optional.empty();
    DCMotor motorSim = null;
    int numberOfMotors = config.getFollowers().map(it -> it.length + 1).orElse(1);
    Object motorController = getMotor(controller, type, id);
    switch (type.toLowerCase()) {
      case "neo":
        motorSim = DCMotor.getNEO(numberOfMotors);
        break;
      case "neo550":
        motorSim = DCMotor.getNeo550(numberOfMotors);
        break;
      case "krakenx60":
        motorSim = DCMotor.getKrakenX60(numberOfMotors);
        break;
      case "krakenx60foc":
        motorSim = DCMotor.getKrakenX60Foc(numberOfMotors);
        break;
      default:
    }

    motor = SmartMotorFactory.create(((SmartMotorController) motorController).getMotorController(), motorSim, config);
    return motor;
  }

  /**
   * Get a motor controller based on the controller type and motor type.
   *
   * @param controller The type of motor controller, e.g. "spark", "talonfx",
   *                   "nova", "thrifty", "thrifty_nova", or "thriftynova"
   * @param type       The type of motor, e.g. "neo", "neo550", "krakenx60", or
   *                   "krakenx44"
   * @param id         The CAN ID of the motor controller
   * @return The motor controller object
   */
  public static Object getReflectedMotor(String controller, String type, int id) {
    ReflectionsManager.VENDOR vendorType = VENDOR.REV;
    switch (controller.toLowerCase()) {
      case "spark":
      case "sparkmax":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.SparkWrapper",
            new Class[] { int.class, SparkBaseType.class }, new Object[] { id, SparkBaseType.SPARK_MAX });
      case "sparkflex":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.SparkWrapper",
            new Class[] { int.class, SparkBaseType.class }, new Object[] { id, SparkBaseType.SPARK_FLEX });
      case "talonfx":
        vendorType = VENDOR.PHOENIX5;
        return ReflectionsManager.<SmartMotorController>create(vendorType,
            "yams.motorcontrollers.remote.TalonFXWrapper",
            new Class[] { int.class }, new Object[] { id });
      case "talonfxs":
        vendorType = VENDOR.PHOENIX6;
        return ReflectionsManager.<SmartMotorController>create(vendorType,
            "yams.motorcontrollers.remote.TalonFXSWrapper",
            new Class[] { int.class }, new Object[] { id });
      case "nova":
      case "thrifty":
      case "thrifty_nova":
      case "thriftynova":
        vendorType = VENDOR.THRIFTYBOT;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.NovaWrapper",
            new Class[] { int.class }, new Object[] { id });
      default:
        throw new RuntimeException("Unknown motor controller type: " + controller);
    }
  }

  /**
   * Method to check the existence of specific JSON configuration files in the
   * provided directory.
   *
   * @param directory the directory to check for JSON configuration files
   */
  public static void checkDirectory(File directory) {
    assert new File(directory, "robot.json").exists();
  }
}
