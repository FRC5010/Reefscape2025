package frc.robot.autos2;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Meter;

public class Right2Coral extends AutoSequence {
  
  public Right2Coral() {
    //Location Definitions
    Pose2d reefCenter = new Pose2d(4.5, 4.0, new Rotation2d());

    // Auto Path Definitions
    AutoPath startToI = AutoPath.PP("Start to I");
    AutoPath iToStation = AutoPath.PP("I to Station");
    AutoPath stationToJ = AutoPath.PP("Station to J");
    AutoPath jToStation = AutoPath.PP("J to Station");

    // Trigger Definitions
    Trigger robotNearReef = robotNearLocation(reefCenter, Meter.of(2.0));

    // Commands
    // TODO: Make actual commands

    // Command Sequence
    addCommands(
      startToI.resetOdometryToStart(), 
      startToI.follow(), 
      iToStation.follow(), 
      stationToJ.follow(), 
      jToStation.follow());
  }

}