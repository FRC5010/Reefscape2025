// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ReefscapeButtonBoard;

/** Add your docs here. */
public class AutoChoosers {
    public static enum ScoringLocations {
        A(ReefscapeButtonBoard.ScoringLocation.FRONT_AB, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        B(ReefscapeButtonBoard.ScoringLocation.FRONT_AB, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT),
        C(ReefscapeButtonBoard.ScoringLocation.FRONT_RIGHT_CD, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        D(ReefscapeButtonBoard.ScoringLocation.FRONT_RIGHT_CD, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT),
        E(ReefscapeButtonBoard.ScoringLocation.BACK_RIGHT_EF, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        F(ReefscapeButtonBoard.ScoringLocation.BACK_RIGHT_EF, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT),
        G(ReefscapeButtonBoard.ScoringLocation.BACK_GH, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        H(ReefscapeButtonBoard.ScoringLocation.BACK_GH, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT),
        I(ReefscapeButtonBoard.ScoringLocation.BACK_LEFT_IJ, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        J(ReefscapeButtonBoard.ScoringLocation.BACK_LEFT_IJ, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT),
        K(ReefscapeButtonBoard.ScoringLocation.FRONT_LEFT_KL, ReefscapeButtonBoard.ScoringAlignment.REEF_LEFT),
        L(ReefscapeButtonBoard.ScoringLocation.FRONT_LEFT_KL, ReefscapeButtonBoard.ScoringAlignment.REEF_RIGHT);

        public ReefscapeButtonBoard.ScoringLocation location;
        public ReefscapeButtonBoard.ScoringAlignment align;
        private ScoringLocations(ReefscapeButtonBoard.ScoringLocation location, ReefscapeButtonBoard.ScoringAlignment align) {
            this.location = location;
            this.align = align;
        }
    }

    public static SendableChooser<ScoringLocations> reef1 = new SendableChooser<>();
    public static SendableChooser<ScoringLocations> reef2 = new SendableChooser<>();
    public static SendableChooser<ScoringLocations> reef3 = new SendableChooser<>();
    public static SendableChooser<ScoringLocations> reef4 = new SendableChooser<>();
    public static SendableChooser<ReefscapeButtonBoard.LoadingStationLocation> station = new SendableChooser<>();
    public static SendableChooser<ReefscapeButtonBoard.ScoringLevel> level = new SendableChooser<>();

    public AutoChoosers(ShuffleboardTab tab) {
        for (ScoringLocations location : ScoringLocations.values()) {
            reef1.addOption(location.name(), location);
            reef2.addOption(location.name(), location);
            reef3.addOption(location.name(), location);
            reef4.addOption(location.name(), location);
        }
        reef1.setDefaultOption(ScoringLocations.F.name(), ScoringLocations.F);
        reef2.setDefaultOption(ScoringLocations.E.name(), ScoringLocations.E);
        reef3.setDefaultOption(ScoringLocations.D.name(), ScoringLocations.D);
        reef4.setDefaultOption(ScoringLocations.C.name(), ScoringLocations.C);
        for (ReefscapeButtonBoard.LoadingStationLocation location : ReefscapeButtonBoard.LoadingStationLocation.values()) {
            station.addOption(location.name(), location);
        }
        station.setDefaultOption(ReefscapeButtonBoard.LoadingStationLocation.STATION_RIGHT_OUTER.name(), ReefscapeButtonBoard.LoadingStationLocation.STATION_RIGHT_OUTER);
        for (ReefscapeButtonBoard.ScoringLevel score : ReefscapeButtonBoard.ScoringLevel.values()) {
            level.addOption(score.name(), score);
        }
        level.setDefaultOption(ReefscapeButtonBoard.ScoringLevel.L4.name(), ReefscapeButtonBoard.ScoringLevel.L4);
        tab.add("Reef 1", reef1).withPosition(8, 0);
        tab.add("Reef 2", reef2).withPosition(8, 2);
        tab.add("Reef 3", reef3).withPosition(8, 4);
        tab.add("Reef 4", reef4).withPosition(8, 6);
        tab.add("Loading Station", station).withPosition(10, 0);
        tab.add("Level", level).withPosition(10, 2);
    }
}
