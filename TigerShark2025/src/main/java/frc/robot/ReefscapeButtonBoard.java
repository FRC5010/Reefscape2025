// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Field;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frc5010.common.sensors.ButtonBoard;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ReefscapeButtonBoard extends ButtonBoard {

    private interface ButtonStateSetting {
        public int getButton();
    }

    private static Transform2d robotOffset = new Transform2d(Meters.of(0.4), Meters.zero(), Rotation2d.fromDegrees(180)); // Move to more appropriate, common, location

    public enum ScoringLocation implements ButtonStateSetting{
        FRONT_AB(0, FieldConstants.Reef.Side.AB.getRobotPose(robotOffset)), // new Pose2d(3.179, 4.035, new Rotation2d(0.0)
        FRONT_RIGHT_CD(1, FieldConstants.Reef.Side.CD.getRobotPose(robotOffset)),
        BACK_RIGHT_EF(2, FieldConstants.Reef.Side.EF.getRobotPose(robotOffset)),
        BACK_GH(3, FieldConstants.Reef.Side.GH.getRobotPose(robotOffset)),
        BACK_LEFT_IJ(4, FieldConstants.Reef.Side.IJ.getRobotPose(robotOffset)),
        FRONT_LEFT_KL(5, FieldConstants.Reef.Side.KL.getRobotPose(robotOffset));

        private int button;
        private Pose2d pose;

        private ScoringLocation(int buttonID, Pose2d pose) {
            this.button = buttonID;
            this.pose = pose;
        }

        public int getButton() {
            return button;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public enum ScoringAlignment implements ButtonStateSetting{
        REEF_LEFT(6, new Transform2d(0.0, 0.16, new Rotation2d())),
        ALGAE(7, new Transform2d()),
        REEF_RIGHT(8, new Transform2d(0.0, -0.16, new Rotation2d()));

        private int button;
        private Transform2d transform;

        private ScoringAlignment(int buttonID, Transform2d transform) {
            this.button = buttonID;
            this.transform = transform;
        }

        public int getButton() {
            return button;
        }

        public Transform2d getTransform2d() {
            return transform;
        }
    }

    public static enum ScoringLevel implements ButtonStateSetting{
        INTAKE(9),
        L1(10),
        L2(11),
        L3(12),
        L4(13),
        NET(14);

        private int button;

        private ScoringLevel(int buttonID) {
            this.button = buttonID;
        }

        public int getButton() {
            return button;
        }
    }

    public enum LoadingStationLocation  implements ButtonStateSetting{

        STATION_LEFT_OUTER(15, FieldConstants.CoralStation.getGuideOffsetPose(FieldConstants.CoralStation.leftCenterFace, -3, robotOffset)),
        STATION_LEFT_INNER(16, FieldConstants.CoralStation.getGuideOffsetPose(FieldConstants.CoralStation.leftCenterFace, 3, robotOffset)),
        STATION_RIGHT_INNER(17, FieldConstants.CoralStation.getGuideOffsetPose(FieldConstants.CoralStation.rightCenterFace, -3, robotOffset)),
        STATION_RIGHT_OUTER(18, FieldConstants.CoralStation.getGuideOffsetPose(FieldConstants.CoralStation.rightCenterFace, 3, robotOffset));


        private int button;
        private Pose2d pose;

        private LoadingStationLocation(int buttonID, Pose2d pose) {
            this.button = buttonID;
            this.pose = pose;
        }

        public int getButton() {
            return button;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public static ScoringLocation scoringLocation = ScoringLocation.BACK_LEFT_IJ;
    public static ScoringAlignment scoringAlignment = ScoringAlignment.REEF_LEFT;
    public static ScoringLevel scoringLevel = ScoringLevel.L1;
    public static LoadingStationLocation loadingStationLocation = LoadingStationLocation.STATION_LEFT_INNER;
    public boolean fineControl = false;
    public ShuffleboardTab tab = Shuffleboard.getTab("Driver");

    public ReefscapeButtonBoard(int port) {
        super(port);
        initializeElasticDisplay();
        
    }

    

    private void initializeElasticDisplay() {
        tab.addBoolean(ScoringLocation.FRONT_AB.name(), () -> ScoringLocation.FRONT_AB == scoringLocation).withPosition(2, 8);
        tab.addBoolean(ScoringLocation.FRONT_RIGHT_CD.name(), () -> ScoringLocation.FRONT_RIGHT_CD == scoringLocation).withPosition(4, 6);
        tab.addBoolean(ScoringLocation.BACK_RIGHT_EF.name(), () -> ScoringLocation.BACK_RIGHT_EF == scoringLocation).withPosition(4, 4);
        tab.addBoolean(ScoringLocation.BACK_GH.name(), () -> ScoringLocation.BACK_GH == scoringLocation).withPosition(2, 2);
        tab.addBoolean(ScoringLocation.BACK_LEFT_IJ.name(), () -> ScoringLocation.BACK_LEFT_IJ == scoringLocation).withPosition(0, 4);
        tab.addBoolean(ScoringLocation.FRONT_LEFT_KL.name(), () -> ScoringLocation.FRONT_LEFT_KL == scoringLocation).withPosition(0, 6);
        tab.addBoolean(ScoringAlignment.REEF_LEFT.name(), () -> ScoringAlignment.REEF_LEFT == scoringAlignment).withPosition(6, 6);
        tab.addBoolean(ScoringAlignment.ALGAE.name(), () -> ScoringAlignment.ALGAE == scoringAlignment).withPosition(8, 4);
        tab.addBoolean(ScoringAlignment.REEF_RIGHT.name(), () -> ScoringAlignment.REEF_RIGHT == scoringAlignment).withPosition(10, 6);
        tab.addBoolean(LoadingStationLocation.STATION_LEFT_OUTER.name(), () -> LoadingStationLocation.STATION_LEFT_OUTER == loadingStationLocation).withPosition(0, 0);
        tab.addBoolean(LoadingStationLocation.STATION_LEFT_INNER.name(), () -> LoadingStationLocation.STATION_LEFT_INNER == loadingStationLocation).withPosition(2, 0);
        tab.addBoolean(LoadingStationLocation.STATION_RIGHT_INNER.name(), () -> LoadingStationLocation.STATION_RIGHT_INNER == loadingStationLocation).withPosition(4, 0);
        tab.addBoolean(LoadingStationLocation.STATION_RIGHT_OUTER.name(), () -> LoadingStationLocation.STATION_RIGHT_OUTER == loadingStationLocation).withPosition(6, 0);
        tab.addBoolean(ScoringLevel.INTAKE.name(), () -> ScoringLevel.INTAKE == scoringLevel).withPosition(12, 8);
        tab.addBoolean(ScoringLevel.L1.name(), () -> ScoringLevel.L1 == scoringLevel).withPosition(12, 6);
        tab.addBoolean(ScoringLevel.L2.name(), () -> ScoringLevel.L2 == scoringLevel).withPosition(12, 4);
        tab.addBoolean(ScoringLevel.L3.name(), () -> ScoringLevel.L3 == scoringLevel).withPosition(12, 2);
        tab.addBoolean(ScoringLevel.L4.name(), () -> ScoringLevel.L4 == scoringLevel).withPosition(12, 0);
        tab.addBoolean(ScoringLevel.NET.name(), () -> ScoringLevel.NET == scoringLevel).withPosition(10, 0);
    }

    private <E extends Enum<E> & ButtonStateSetting> void bindStateSettorButtons(Consumer<E> setter, E[] buttonEnums) {
        for (E enumInstance : buttonEnums) {
            getButton(enumInstance.getButton()).onTrue(Commands.runOnce(() -> setter.accept(enumInstance)));
        }
    }

    private void setupButtonBindings() {
        bindStateSettorButtons(ReefscapeButtonBoard::setScoringLocation, ScoringLocation.values());
        bindStateSettorButtons(ReefscapeButtonBoard::setAlignment, ScoringAlignment.values());
        bindStateSettorButtons(ReefscapeButtonBoard::setScoringLevel, ScoringLevel.values());
        bindStateSettorButtons(ReefscapeButtonBoard::setLoadingStation, LoadingStationLocation.values());
    }

    public void configureOperatorButtonBindings(Controller operator) {
        operator.createDownPovButton().onTrue(Commands.runOnce(() -> setScoringLevel(ScoringLevel.INTAKE)));
        operator.createAButton().onTrue(Commands.runOnce(() -> setScoringLevel(ScoringLevel.L1)));
        operator.createXButton().onTrue(Commands.runOnce(() -> setScoringLevel(ScoringLevel.L2)));
        operator.createYButton().onTrue(Commands.runOnce(() -> setScoringLevel(ScoringLevel.L3)));
        operator.createBButton().onTrue(Commands.runOnce(() -> setScoringLevel(ScoringLevel.L4)));

        
        operator.createLeftPovButton().onTrue(Commands.runOnce(() -> setAlignment(ScoringAlignment.REEF_LEFT)));
        operator.createUpPovButton().onTrue(Commands.runOnce(() -> setAlignment(ScoringAlignment.ALGAE)));
        operator.createRightPovButton().onTrue(Commands.runOnce(() -> setAlignment(ScoringAlignment.REEF_RIGHT)));

        operator.createRightBumper().onTrue(Commands.runOnce(() -> {
            setLoadingStation(LoadingStationLocation.values()[loadingStationLocation.ordinal() >= LoadingStationLocation.values().length - 1 ? 0 : loadingStationLocation.ordinal() + 1]);
        }));
        operator.createLeftBumper().onTrue(Commands.runOnce(() -> {
            setLoadingStation(LoadingStationLocation.values()[loadingStationLocation.ordinal() <= 0 ? LoadingStationLocation.values().length - 1  : loadingStationLocation.ordinal() - 1]);
        }));

        operator.createBackButton().onTrue(Commands.runOnce(() -> {
            setScoringLocation(ScoringLocation.values()[scoringLocation.ordinal() >= ScoringLocation.values().length - 1 ? 0 : scoringLocation.ordinal() + 1]);
        }));

        operator.createStartButton().onTrue(Commands.runOnce(() -> {
            setScoringLocation(ScoringLocation.values()[scoringLocation.ordinal() <= 0 ? ScoringLocation.values().length - 1  : scoringLocation.ordinal() - 1]);
        }));

    }


    public static ScoringLocation getScoringLocation() {
        return scoringLocation;

    }

    public static void setScoringLocation(ScoringLocation location) {
        scoringLocation = location;
    }

    public static ScoringAlignment getCurrentAlignment() {
        return scoringAlignment;
    }

    public static void setAlignment(ScoringAlignment alignment) {
        scoringAlignment = alignment;
    }

    public static ScoringLevel getScoringLevel() {
        return scoringLevel;
    }

    public static void setScoringLevel(ScoringLevel level) {
        scoringLevel = level;
    }

    public static LoadingStationLocation getLoadingStation() {
        return loadingStationLocation;
    }

    public static Pose2d getStationPose() {
        return loadingStationLocation.getPose();
    }

    public static void setLoadingStation(LoadingStationLocation loadingLocation) {
        loadingStationLocation = loadingLocation;
    }

    public static Pose2d getScoringPose() {
        return scoringLocation.getPose().transformBy(scoringAlignment.getTransform2d());
    }

    public static Supplier<Pose2d> getScoringPoseSupplier() {
        return () -> getScoringPose();
    }

}
