// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class TargetingSystem {

    public static YAGSLSwerveDrivetrain drivetrain;
    public static ShooterSystem shooter;
    public static ElevatorSystem elevator;
    public static AlgaeArm arm;

    public static Supplier<Distance> maxHeight;

    private static Distance MAX_NO_TIPPY_HEIGHT = Meters.of(0.5);
    private static Transform2d CoralOffset = new Transform2d(-0.05, 0.0, new Rotation2d());
    private static Transform2d StationOffset = new Transform2d(0.0, 0.0, new Rotation2d());

    public static void setupParameters(YAGSLSwerveDrivetrain drivetrain, ShooterSystem shooter, ElevatorSystem elevator,
            AlgaeArm arm) {
        TargetingSystem.drivetrain = drivetrain;
        TargetingSystem.shooter = shooter;
        TargetingSystem.elevator = elevator;
        TargetingSystem.arm = arm;
    }

    public static void setupParameters(YAGSLSwerveDrivetrain drivetrain, ShooterSystem shooter,
            ElevatorSystem elevator) {
        TargetingSystem.drivetrain = drivetrain;
        TargetingSystem.shooter = shooter;
        TargetingSystem.elevator = elevator;
    }

    public static YAGSLSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private static double getDrivetrainSpeed() {
        return new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond,
                drivetrain.getChassisSpeeds().vyMetersPerSecond).getNorm();
    }

    private static Supplier<Distance> getMaxHeightSupplier(Pose2d targetPose) {
        return () -> {
            if (targetPose.getTranslation().getDistance(drivetrain.getPose().getTranslation()) < 1.0
                    || getDrivetrainSpeed() > 0.5) {
                return MAX_NO_TIPPY_HEIGHT;
            } else {
                return Meters.of(2);
            }
        };
    }

    public static Command createCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);

        return drivetrain.driveToPosePrecise(targetPose, CoralOffset).get()
                .until(() -> elevator.getElevatorPosition().in(Meters) > 1.0)
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocationImproved(level)));
    }

    public static Command createAutoCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance prescoreLevel = Meters.of(0.6);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        return drivetrain.driveToPoseAuton(() -> targetPose, CoralOffset, Seconds.of(5)).get().raceWith(elevator.pidControlCommand(prescoreLevel))
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))
                        .andThen(shooter.getShootCommand(scoringLevel)).until(shooter.isEmpty()));
    }

    public static Command createLoadingSequence(Pose2d targetPose) {
        Distance level = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);

        return drivetrain.driveToPosePrecise(targetPose, StationOffset).get()
                .alongWith(Commands.runOnce(() -> SmartDashboard.putNumber("Elevator Level:", level.in(Meters))),
                        elevator.pidControlCommand(level));
    }

    public static Command createAutoLoadingSequence(Pose2d targetPose) {
        Distance l3Level = elevator.selectElevatorLevel(() -> ScoringLevel.L3);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);

        return Commands.sequence(
                elevator.pidControlCommand(intakeLevel).until(() -> elevator.isAtLocation(intakeLevel))
                .andThen(elevator.elevatorPositionZeroSequence())
                .andThen(elevator.holdElevatorDown())
                        )
                .alongWith(
                        Commands.idle().until(() -> elevator.getElevatorPosition().in(Meters) < l3Level.in(Meters)+0.5).andThen(
                        Commands.parallel(
                                drivetrain.driveToPoseAuton(() -> targetPose, StationOffset, Seconds.of(3))
                                        .get(),
                                shooter.intakeCoral()
                                        )).until(shooter.coralCapturedOrAligned())).until(shooter.coralCapturedOrAligned()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command driveXMetersQuest(Distance distance) {
        return drivetrain.driveToPose(
                drivetrain.getPose().transformBy(new Transform2d(distance.in(Meters), 0.0, drivetrain.getHeading())));
    }
}
