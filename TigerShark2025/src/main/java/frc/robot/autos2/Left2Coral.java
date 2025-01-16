package frc.robot.autos2;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Meter;

public class Left2Coral extends AutoSequence {
  
  public Left2Coral() {
    //Location Definitions
    Pose2d reefCenter = new Pose2d(4.5, 4.0, new Rotation2d());

    // Auto Path Definitions
    AutoPath startToF = AutoPath.Choreo("Start to F");
    AutoPath fToStation = AutoPath.Choreo("F to Station");
    AutoPath stationToE = AutoPath.Choreo("Station to E");
    AutoPath eToStation = AutoPath.Choreo("E to Station");

    // Trigger Definitions
    Trigger robotNearReef = robotNearLocation(reefCenter, Meter.of(2.0));

    // Commands
    // TODO: Make actual commands

    // Command Sequence
    addCommands(
      startToF.resetOdometryToStart(0.1), 
      startToF.follow(), 
      fToStation.follow(), 
      stationToE.follow(), 
      eToStation.follow());
  }

}
