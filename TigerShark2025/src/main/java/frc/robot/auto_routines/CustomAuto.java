// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import java.util.function.Supplier;

import org.frc5010.common.auto.AutoSequence;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.LoadingStationLocation;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.auto_routines.AutoChoosers.ScoringLocations;
import frc.robot.managers.TargetingSystem;

/** Add your docs here. */
public class CustomAuto extends AutoSequence {
  private Command scoreCoral(Supplier<ScoringLocation> location, Supplier<ScoringAlignment> alignment, Supplier<ScoringLevel> level) {
    return Commands.deferredProxy(() ->TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(location.get(), alignment.get()), level.get()));
  }

  private Command scoreCoralNotDefereed(Supplier<ScoringLocation> location, Supplier<ScoringAlignment> alignment, Supplier<ScoringLevel> level) {
    return TargetingSystem.createAutoCoralScoringSequence(
      ReefscapeButtonBoard.getScoringPose(location.get(), alignment.get()), level.get());
  }

  private Command scoreCoral(SendableChooser<ScoringLocations> chooser, SendableChooser<ScoringLevel> level) {
    return scoreCoral(() -> chooser.getSelected().location, () -> chooser.getSelected().align, () -> level.getSelected());
  }

  private Command loadCoral(Supplier<LoadingStationLocation> location) {
    return Commands.deferredProxy(() -> TargetingSystem.createAutoLoadingSequence(
      ReefscapeButtonBoard.getLoadingPose(location.get())));
  }

  private Command loadCoral(SendableChooser<LoadingStationLocation> chooser) {
    return loadCoral(() -> chooser.getSelected());
  }
  

  public CustomAuto() {
        addCommands(
          scoreCoral(AutoChoosers.reef1, AutoChoosers.level),
          loadCoral(AutoChoosers.station),
          scoreCoral(AutoChoosers.reef2, AutoChoosers.level),
          loadCoral(AutoChoosers.station),
          scoreCoral(AutoChoosers.reef3, AutoChoosers.level),
          loadCoral(AutoChoosers.station),
          scoreCoral(AutoChoosers.reef4, AutoChoosers.level),
          loadCoral(AutoChoosers.station)
        );
  }
}
