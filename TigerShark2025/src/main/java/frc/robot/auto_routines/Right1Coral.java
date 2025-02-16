// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class Right1Coral extends AutoSequence {
    public Right1Coral(YAGSLSwerveDrivetrain yagsl, ShooterSystem shooter, ElevatorSystem elevator) {


        // Command Sequence
        addCommands(
        TargetingSystem.createCoralScoringSequence(ReefscapeButtonBoard.getScoringPose(ScoringLocation.BACK_GH, ScoringAlignment.REEF_RIGHT), ScoringLevel.L4)
        );
  }
}
