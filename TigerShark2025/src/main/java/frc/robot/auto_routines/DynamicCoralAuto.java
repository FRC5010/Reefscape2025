// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import static edu.wpi.first.units.Units.Meter;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class DynamicCoralAuto extends AutoSequence {

    public DynamicCoralAuto() {
        //Location Definitions
        Pose2d reefCenter = new Pose2d(4.5, 4.0, new Rotation2d());

        // Auto Path Definitions
        AutoPath startToI = AutoPath.PP("Start 3 to I");
        AutoPath iToStation = AutoPath.PP("I to T1");
        AutoPath stationToJ = AutoPath.PP("T1 to J");
        AutoPath jToStation = AutoPath.PP("J to T1");
        AutoPath stationToK = AutoPath.PP("T1 to K");
        AutoPath kToStation = AutoPath.PP("K to T1");
        AutoPath stationToL = AutoPath.PP("T1 to L");
        AutoPath LToStation = AutoPath.PP("L to T1");
        AutoPath T1ToStart3 = AutoPath.PP("T1 to Start 3");

        // Trigger Definitions
        Trigger robotNearReef = robotNearLocation(reefCenter, Meter.of(2.0));
        // Commands
        // TODO: Make actual commands

        // Command Sequence
        addCommands(
        startToI.resetOdometryToStart(0.5), 
        startToI.follow(), 
        iToStation.follow(), 
        stationToJ.follow(), 
        jToStation.follow(),
        stationToK.follow(),
        kToStation.follow(),
        stationToL.follow(),
        LToStation.follow(),
        T1ToStart3.follow());
    }
}
