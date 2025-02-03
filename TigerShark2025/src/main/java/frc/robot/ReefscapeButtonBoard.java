// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5010.common.sensors.ButtonBoard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class ReefscapeButtonBoard extends ButtonBoard {

    public enum ScoringLocation{
        FRONT_AB,
        FRONT_RIGHT_CD,
        BACK_RIGHT_EF,
        BACK_GH,
        BACK_LEFT_IJ,
        FRONT_LEFT_KL
    }

    public enum ScoringAlignment{
        REEF_LEFT,
        ALGAE,
        REEF_RIGHT
    }

    public enum ScoringLevel{
        INTAKE,
        L1,
        L2,
        L3,
        L4,
        NET
    }

    public enum LoadingStatingLocation{
        STATION_LEFT,
        STATION_RIGHT
    }

    public static ScoringLocation scoringLocation = ScoringLocation.FRONT_AB;
    public static ScoringAlignment scoringAlignment = ScoringAlignment.REEF_LEFT;
    public static ScoringLevel scoringLevel = ScoringLevel.L1;
    public static LoadingStatingLocation loadingStatingLocation = LoadingStatingLocation.STATION_LEFT;
    public boolean fineControl = false;
    public ShuffleboardTab tab = Shuffleboard.getTab("Driver");

    public ReefscapeButtonBoard(int port) {
        super(port);
        tab.addBoolean(ScoringLocation.FRONT_AB.name(), () -> ScoringLocation.FRONT_AB == scoringLocation).withPosition(2, 8);
        tab.addBoolean(ScoringLocation.FRONT_RIGHT_CD.name(), () -> ScoringLocation.FRONT_RIGHT_CD == scoringLocation).withPosition(4, 6);
        tab.addBoolean(ScoringLocation.BACK_RIGHT_EF.name(), () -> ScoringLocation.BACK_RIGHT_EF == scoringLocation).withPosition(4, 4);
        tab.addBoolean(ScoringLocation.BACK_GH.name(), () -> ScoringLocation.BACK_GH == scoringLocation).withPosition(2, 2);
        tab.addBoolean(ScoringLocation.BACK_LEFT_IJ.name(), () -> ScoringLocation.BACK_LEFT_IJ == scoringLocation).withPosition(0, 4);
        tab.addBoolean(ScoringLocation.FRONT_LEFT_KL.name(), () -> ScoringLocation.FRONT_LEFT_KL == scoringLocation).withPosition(0, 6);
        tab.addBoolean(ScoringAlignment.REEF_LEFT.name(), () -> ScoringAlignment.REEF_LEFT == scoringAlignment).withPosition(6, 6);
        tab.addBoolean(ScoringAlignment.ALGAE.name(), () -> ScoringAlignment.ALGAE == scoringAlignment).withPosition(8, 8);
        tab.addBoolean(ScoringAlignment.REEF_RIGHT.name(), () -> ScoringAlignment.REEF_RIGHT == scoringAlignment).withPosition(10, 6);
        tab.addBoolean(LoadingStatingLocation.STATION_LEFT.name(), () -> LoadingStatingLocation.STATION_LEFT == loadingStatingLocation).withPosition(0, 0);
        tab.addBoolean(LoadingStatingLocation.STATION_RIGHT.name(), () -> LoadingStatingLocation.STATION_RIGHT == loadingStatingLocation).withPosition(4, 0);
        tab.addBoolean(ScoringLevel.INTAKE.name(), () -> ScoringLevel.INTAKE == scoringLevel).withPosition(12, 8);
        tab.addBoolean(ScoringLevel.L1.name(), () -> ScoringLevel.L1 == scoringLevel).withPosition(12, 6);
        tab.addBoolean(ScoringLevel.L2.name(), () -> ScoringLevel.L2 == scoringLevel).withPosition(12, 4);
        tab.addBoolean(ScoringLevel.L3.name(), () -> ScoringLevel.L3 == scoringLevel).withPosition(12, 2);
        tab.addBoolean(ScoringLevel.L4.name(), () -> ScoringLevel.L4 == scoringLevel).withPosition(12, 0);
        tab.addBoolean(ScoringLevel.NET.name(), () -> ScoringLevel.NET == scoringLevel).withPosition(10, 0);
    }

}
