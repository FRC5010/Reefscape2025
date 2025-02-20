// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.LoadingStationLocation;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class Coral2 extends AutoSequence {
    public Coral2(YAGSLSwerveDrivetrain yagsl, ShooterSystem shooter, ElevatorSystem elevator) {


        // Command Sequence
        addCommands(
        TargetingSystem.createAutoCoralScoringSequence(ReefscapeButtonBoard.getScoringPose(ScoringLocation.BACK_RIGHT_EF, ScoringAlignment.REEF_RIGHT), ScoringLevel.L4),
        TargetingSystem.createAutoLoadingSequence(ReefscapeButtonBoard.getLoadingPose(LoadingStationLocation.STATION_RIGHT_OUTER)),
        TargetingSystem.createAutoCoralScoringSequence(ReefscapeButtonBoard.getScoringPose(ScoringLocation.FRONT_RIGHT_CD, ScoringAlignment.REEF_RIGHT), ScoringLevel.L4)
        

        );
  }
}
