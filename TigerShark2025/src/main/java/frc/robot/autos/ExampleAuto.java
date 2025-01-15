// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static edu.wpi.first.units.Units.Meter;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Add your docs here. */
public class ExampleAuto extends AutoSequence {


    public ExampleAuto() {
        AutoPath questTest = AutoPath.PP("Quest Test Path 1");

        robotNearLocation(new Pose2d(5, 5, new Rotation2d()), Meter.of(1))
            .onTrue(new PrintCommand("I'm near the reef"));
        
        addCommands(
            questTest.resetOdometryToStart(),
            questTest.follow()
        );
    }


}
