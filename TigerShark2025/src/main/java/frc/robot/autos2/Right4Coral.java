package frc.robot.autos2;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Meter;

public class Right4Coral extends AutoSequence {
  
  public Right4Coral() {
    //Location Definitions
    Pose2d reefCenter = new Pose2d(4.5, 4.0, new Rotation2d());

    // Auto Path Definitions
    AutoPath startToI = AutoPath.PP("Start to I");
    AutoPath iToStation = AutoPath.PP("I to Station");
    AutoPath stationToJ = AutoPath.PP("Station to J");
    AutoPath jToStation = AutoPath.PP("J to Station");
    AutoPath stationToK = AutoPath.PP("Station to K");
    AutoPath kToStation = AutoPath.PP("K to Station");
    AutoPath stationToL = AutoPath.PP("Station to L");
    AutoPath LToStation = AutoPath.PP("L to Station");

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
      jToStation.follow(),
      stationToK.follow(),
      kToStation.follow(),
      stationToL.follow(),
      LToStation.follow());
  }

}
