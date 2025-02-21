// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class CustomAuto extends AutoSequence {
  public CustomAuto(YAGSLSwerveDrivetrain yagsl, ShooterSystem shooter, ElevatorSystem elevator) {
        // Command Sequence
        addCommands(
        TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(AutoChoosers.reef1.getSelected().location, AutoChoosers.reef1.getSelected().align), AutoChoosers.level.getSelected()),
        TargetingSystem.createAutoLoadingSequence(
          ReefscapeButtonBoard.getLoadingPose(AutoChoosers.station.getSelected())),
        TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(AutoChoosers.reef2.getSelected().location, AutoChoosers.reef2.getSelected().align), ScoringLevel.L4)
        ,
        TargetingSystem.createAutoLoadingSequence(
          ReefscapeButtonBoard.getLoadingPose(AutoChoosers.station.getSelected())),
        TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(AutoChoosers.reef3.getSelected().location, AutoChoosers.reef3.getSelected().align), ScoringLevel.L4)
        ,
        TargetingSystem.createAutoLoadingSequence(
          ReefscapeButtonBoard.getLoadingPose(AutoChoosers.station.getSelected())),
        TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(AutoChoosers.reef4.getSelected().location, AutoChoosers.reef4.getSelected().align), ScoringLevel.L4)
        );
  }
}
