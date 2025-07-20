package org.frc5010.common.config;

import java.io.File;
import java.io.IOException;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.config.json.devices.SubsystemJson;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

public class SubsystemParser {
  /** The mechanism simulation */
  protected LoggedMechanism2d mechanismSimulation;
  /** The directory to read from */
  protected String robotDirectory;
  /** The robot */
  protected GenericRobot robot;

  /**
   * Creates a new RobotParser.
   *
   * @param robotDirectory the directory to read from
   * @param robot the robot being configured
   * @throws IOException
   */
  public SubsystemParser(String robotDirectory, GenericRobot robot) throws IOException {
    mechanismSimulation = robot.getMechVisual();
    this.robotDirectory = robotDirectory;
    this.robot = robot;
  }

/**
 * Checks if a specific configuration file exists in the given directory.
 *
 * @param directory the directory to check for the configuration file
 * @param configFile the name of the configuration file to verify existence
 */
  private void checkDirectory(File directory, String configFile) {
    assert new File(directory, configFile).exists();
  }

  /**
   * Parses a subsystem configuration from the given file and configures the subsystem.
   *
   * @param genericSubsystem the subsystem to configure
   * @param configFile the name of the configuration file to read
   * @throws StreamReadException if the file cannot be read
   * @throws DatabindException if the file cannot be parsed
   * @throws IOException if there is an error reading the file
   */
  public void parseSubsystem(GenericSubsystem genericSubsystem, String configFile)
      throws StreamReadException, DatabindException, IOException {
    File directory = new File(Filesystem.getDeployDirectory(), robotDirectory + "/subsystems/");
    checkDirectory(directory, configFile);

    genericSubsystem.setMechSimulation(mechanismSimulation);

    // Read in the subsystem configuration
    // Read in the robot configuration
    SubsystemJson subsystemJson =
        new ObjectMapper().readValue(new File(directory, configFile), SubsystemJson.class);
    genericSubsystem.setDisplay(subsystemJson.display);
    genericSubsystem.setLoggingLevel(LogLevel.valueOf(subsystemJson.logLevel));
    subsystemJson.configureSubsystem(genericSubsystem, directory);
  }
}
