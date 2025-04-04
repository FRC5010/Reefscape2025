// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.commands.calibration.WheelRadiusCharacterization;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class WheelCalibrationAuto extends AutoSequence  {
    public WheelCalibrationAuto(YAGSLSwerveDrivetrain drivetrain) {
        Command wheelCalibrationCommand = new WheelRadiusCharacterization(drivetrain);
        addCommands(wheelCalibrationCommand);
    }
}
