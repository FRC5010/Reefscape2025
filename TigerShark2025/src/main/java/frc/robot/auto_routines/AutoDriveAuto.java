// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto_routines;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Seconds;

import org.frc5010.common.auto.AutoPath;
import org.frc5010.common.auto.AutoSequence;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;
import frc.robot.ReefscapeButtonBoard.ScoringAlignment;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class AutoDriveAuto extends AutoSequence {
    public AutoDriveAuto(YAGSLSwerveDrivetrain yagsl, ShooterSystem shooter, ElevatorSystem elevator) {


        // Command Sequence
        addCommands(
        yagsl.driveToPosePrecise(ReefscapeButtonBoard.getScoringPose(ScoringLocation.BACK_GH, ScoringAlignment.REEF_LEFT)).get(),
        Commands.waitTime(Seconds.of(0.2)),
        elevator.profiledBangBangCmd(ElevatorSystem.Position.L3.position()).until(() -> elevator.isAtLocation(ElevatorSystem.Position.L3.position())),
        Commands.waitTime(Seconds.of(0.2)),
        shooter.runMotors(() -> 1).withTimeout(Seconds.of(2))
        );
  }
}
