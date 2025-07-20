// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Elevator;

/** Add your docs here. */
public class ElevatorParser {
    /**
     * Parses an elevator configuration from the specified JSON file and configures the elevator subsystem.
     *
     * @param subDirectory the subdirectory containing the configuration file
     * @param filename the name of the configuration file
     * @param system the subsystem to configure
     * @return the configured Elevator instance, or null if an error occurs
     */

    public static Elevator parse(String subDirectory, String filename, SubsystemBase system) 
    {
        try {
            File directory = new File(Filesystem.getDeployDirectory(), subDirectory);
            DeviceConfigReader.checkDirectory(directory);
            File deviceFile = new File(directory, filename);
            YamsElevatorConfigurationJson yamsElevatorConfigurationJson = new ObjectMapper().readValue(deviceFile,
                    YamsElevatorConfigurationJson.class);
            return yamsElevatorConfigurationJson.configure(system);
        } catch (Exception e) {
            System.out.println("Error reading device configuration: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}
