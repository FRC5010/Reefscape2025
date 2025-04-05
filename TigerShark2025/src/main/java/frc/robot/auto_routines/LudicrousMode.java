// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.auto.pathplanner.PathfindingCommand5010;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.LoadingStationLocation;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.auto_routines.AutoChoosers.ScoringLocations;
import frc.robot.managers.TargetingSystem;

/** Add your docs here. */
public class LudicrousMode extends AutoSequence {
  RobotConfig robotConfig;
  PathConstraints autoConstraints;


  private Command scoreCoral(Supplier<ScoringLocation> location, Supplier<ScoringAlignment> alignment, Supplier<ScoringLevel> level) {
    return Commands.deferredProxy(() ->TargetingSystem.createAutoCoralScoringSequence(
          ReefscapeButtonBoard.getScoringPose(location.get(), alignment.get()), level.get()));
  }
  
  private Command scoreCoralNotDefereed(SendableChooser<ScoringLocations> chooser, SendableChooser<ScoringLevel> level) {
    return TargetingSystem.createAutoCoralScoringSequence(
      ReefscapeButtonBoard.getScoringPose(chooser.getSelected().location, chooser.getSelected().align), level.getSelected());
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

    public LudicrousMode() {
        try {
          robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // A default config in case the GUI settings can't be loaded
          robotConfig = new RobotConfig(Kilogram.of(68).magnitude(),
              SingleJointedArmSim.estimateMOI(0.5, Kilogram.of(68).magnitude()),
              new ModuleConfig(0.1, 4.5, 1.19, 
              DCMotor.getNEO(1), 40, 4), 0.5);
        }

        autoConstraints = new PathConstraints(TargetingSystem.getDrivetrain().getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond() * 1.0,
        8.0,
        TargetingSystem.getDrivetrain().getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(),
        TargetingSystem.getDrivetrain().getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

        Trigger autoAndDisabled = new Trigger(() -> RobotState.isAutonomous() && RobotState.isDisabled());
        
        

        Trigger pathfindingCommandRunning = new Trigger(PathfindingCommand5010::isCommandRunning);
  

        Supplier<Command> pregenPath = () -> Commands.run(() -> 
        {
          Pose2d endPose = AutoChoosers.pathfindingPose.get();
          SmartDashboard.putString("Path End AutoGen", endPose.toString());
          PathfindingCommand5010.setPregenPathfinding(TargetingSystem.getDrivetrain().getPose(), endPose);
          PathfindingCommand5010.updatePregen(autoConstraints, TargetingSystem.getDrivetrain().getPose(), new GoalEndState(0, endPose.getRotation()), robotConfig);
        });
        pregenPath.get().ignoringDisable(true).until(RobotState::isEnabled).schedule();

        
        addCommands(
            scoreCoral(AutoChoosers.reef1, AutoChoosers.level).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setLoadingSupplier(AutoChoosers.station))
                .andThen(pregenPath.get())
            ),
            loadCoral(AutoChoosers.station).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setSupplierScoring(AutoChoosers.reef2))
                .andThen(pregenPath.get())
            ),
            scoreCoral(AutoChoosers.reef2, AutoChoosers.level).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setLoadingSupplier(AutoChoosers.station))
                .andThen(pregenPath.get())
            ),
            loadCoral(AutoChoosers.station).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setSupplierScoring(AutoChoosers.reef3))
                .andThen(pregenPath.get())
            ),
            scoreCoral(AutoChoosers.reef3, AutoChoosers.level).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setLoadingSupplier(AutoChoosers.station))
                .andThen(pregenPath.get())
            ),
            loadCoral(AutoChoosers.station).raceWith(
              Commands.waitSeconds(0.1)
                .andThen(Commands.waitUntil(pathfindingCommandRunning.negate()))
                .andThen(AutoChoosers.setSupplierScoring(AutoChoosers.reef4))
                .andThen(pregenPath.get())
            ),
            scoreCoral(AutoChoosers.reef4, AutoChoosers.level)
          );
    }
}
