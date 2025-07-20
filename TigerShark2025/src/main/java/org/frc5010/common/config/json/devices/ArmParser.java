// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import java.io.File;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;

/** Add your docs here. */
public class ArmParser {
    /**
     * Parses a YamsArm configuration from the given file and adds it to the system.
     *
     * @param subDirectory the subdirectory of the configuration file, relative to the deploy directory
     * @param filename     the name of the configuration file to read
     * @param system       the system to add the device to
     * @return the parsed Arm, or null if the file cannot be read or parsed
     * @throws StreamReadException if the file cannot be read
     * @throws DatabindException   if the file cannot be parsed
     * @throws IOException         if there is an error reading the file
     */
    public static Arm parse(String subDirectory, String filename, SubsystemBase system) 
    {
        try {
            File directory = new File(Filesystem.getDeployDirectory(), subDirectory);
            DeviceConfigReader.checkDirectory(directory);
            File deviceFile = new File(directory, filename);
            YamsArmConfigurationJson yamsArmConfigurationJson = new ObjectMapper().readValue(deviceFile,
                    YamsArmConfigurationJson.class);
            return yamsArmConfigurationJson.configure(system);
        } catch (Exception e) {
            System.out.println("Error reading device configuration: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }
}
