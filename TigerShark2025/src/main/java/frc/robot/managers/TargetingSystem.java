// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard.ScoringLevel;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

/** Add your docs here. */
public class TargetingSystem {

    public static GenericSwerveDrivetrain drivetrain;
    public static ShooterSystem shooter;
    public static ElevatorSystem elevator;
    public static AlgaeArm arm;

    public static Supplier<Distance> maxHeight;

    private static Distance MAX_NO_TIPPY_HEIGHT = Meters.of(0.5);
    private static Transform2d CoralOffset = new Transform2d(-0.02, 0.0, new Rotation2d());
    private static Transform2d StationOffset = new Transform2d(0.0, 0.0, new Rotation2d());

    public static void setupParameters(GenericSwerveDrivetrain drivetrain, ShooterSystem shooter,
            ElevatorSystem elevator,
            AlgaeArm arm) {
        TargetingSystem.drivetrain = drivetrain;
        TargetingSystem.shooter = shooter;
        TargetingSystem.elevator = elevator;
        TargetingSystem.arm = arm;
    }

    public static void setupParameters(GenericSwerveDrivetrain drivetrain, ShooterSystem shooter,
            ElevatorSystem elevator) {
        TargetingSystem.drivetrain = drivetrain;
        TargetingSystem.shooter = shooter;
        TargetingSystem.elevator = elevator;
    }

    public static GenericSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private static double getDrivetrainSpeed() {
        return new Translation2d(drivetrain.getFieldVelocity().vxMetersPerSecond,
                drivetrain.getFieldVelocity().vyMetersPerSecond).getNorm();
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

    public static Command createTeleopCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance prescoreLevel = Meters.of(0.6);
        Distance finalLineupLevel = Meters.of(1.2);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        BooleanSupplier closeAndSlow = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.5)
                && Math.abs(drivetrain.getFieldVelocity().vxMetersPerSecond) < 1.0;
        BooleanSupplier notCloseAndSlow = () -> !closeAndSlow.getAsBoolean();
        BooleanSupplier closeEnough = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.3)
                && Math.abs(drivetrain.getFieldVelocity().vxMetersPerSecond) < 0.5;
        BooleanSupplier elevatorNotAboveFinalLineupLevel = () -> elevator.getElevatorPosition()
                .in(Meters) < finalLineupLevel.in(Meters);
        Trigger closeEnoughOrNotAboveFinalLineupLevel = new Trigger(
                () -> closeEnough.getAsBoolean() || elevatorNotAboveFinalLineupLevel.getAsBoolean());
        return drivetrain.driveToPoseAuton(() -> targetPose, CoralOffset, Seconds.of(5)).get().raceWith(
                elevator.pidControlCommand(prescoreLevel).until(closeAndSlow)
                        .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))
                                .onlyWhile(closeEnough))
                        .andThen(Commands.idle()))
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level)));
    }

    public static Command createAutoCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance prescoreLevel = Meters.of(0.6);
        Distance finalLineupLevel = Meters.of(1.2);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        double maxAcceleration = 4.3;
        Supplier<Translation2d> vectorToTarget = () -> targetPose.getTranslation()
                .minus(drivetrain.getPose().getTranslation());
        DoubleSupplier relativeAngleTowardsTarget = () -> Math
                .acos(((vectorToTarget.get().getX() * drivetrain.getFieldVelocity().vxMetersPerSecond)
                        + (vectorToTarget.get().getY() * drivetrain.getFieldVelocity().vyMetersPerSecond))
                        / (vectorToTarget.get().getNorm()
                                * Math.pow(Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                                        + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2), 0.5)));
        DoubleSupplier stoppingDistance = () -> ((Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2))
                * Math.pow(Math.cos(relativeAngleTowardsTarget.getAsDouble()), 2)) / (2 * maxAcceleration);
        DoubleSupplier distanceToTarget = () -> targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation());
        BooleanSupplier close = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.25); // &&
                                                                             // Math.abs(drivetrain.getChassisSpeeds().vxMetersPerSecond)
                                                                             // < 1.0;
        BooleanSupplier notCloseAndSlow = () -> !close.getAsBoolean();
        BooleanSupplier closeEnough = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.5)
                && Math.abs(drivetrain.getFieldVelocity().vxMetersPerSecond) < 0.5;
        BooleanSupplier elevatorNotAboveFinalLineupLevel = () -> elevator.getElevatorPosition()
                .in(Meters) < finalLineupLevel.in(Meters);
        Trigger closeEnoughOrNotAboveFinalLineupLevel = new Trigger(
                () -> closeEnough.getAsBoolean() || elevatorNotAboveFinalLineupLevel.getAsBoolean());
        return drivetrain
                .newDriveToPoseAuton(() -> targetPose, CoralOffset, Seconds.of(5), maxAcceleration, distanceToTarget,
                        stoppingDistance)
                .get().raceWith(
                        elevator.newPidControlCommand(prescoreLevel).until(close)
                                .andThen(elevator.newPidControlCommand(level)
                                        .until(() -> elevator.getElevatorPosition().in(Meters) > Math.min(
                                                level.in(Meters), ElevatorSystem.Position.L3.position().in(Meters)))
                                        .onlyWhile(close))
                                .andThen(Commands.idle()))
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))
                        .andThen(shooter.getShootCommand(scoringLevel)).until(shooter.isEmpty()));
    }

    public static Command createOneCoralAutoScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance prescoreLevel = Meters.of(0.6);
        Distance finalLineupLevel = Meters.of(1.2);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        double maxAcceleration = 4.3;
        Supplier<Translation2d> vectorToTarget = () -> targetPose.getTranslation()
                .minus(drivetrain.getPose().getTranslation());
        DoubleSupplier relativeAngleTowardsTarget = () -> Math
                .acos(((vectorToTarget.get().getX() * drivetrain.getFieldVelocity().vxMetersPerSecond)
                        + (vectorToTarget.get().getY() * drivetrain.getFieldVelocity().vyMetersPerSecond))
                        / (vectorToTarget.get().getNorm()
                                * Math.pow(Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                                        + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2), 0.5)));
        DoubleSupplier stoppingDistance = () -> ((Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2))
                * Math.pow(Math.cos(relativeAngleTowardsTarget.getAsDouble()), 2)) / (2 * maxAcceleration);
        DoubleSupplier distanceToTarget = () -> targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation());
        BooleanSupplier close = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.25); // &&
                                                                             // Math.abs(drivetrain.getChassisSpeeds().vxMetersPerSecond)
                                                                             // < 1.0;
        BooleanSupplier notCloseAndSlow = () -> !close.getAsBoolean();
        BooleanSupplier closeEnough = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.5)
                && Math.abs(drivetrain.getFieldVelocity().vxMetersPerSecond) < 0.5;
        BooleanSupplier elevatorNotAboveFinalLineupLevel = () -> elevator.getElevatorPosition()
                .in(Meters) < finalLineupLevel.in(Meters);
        Trigger closeEnoughOrNotAboveFinalLineupLevel = new Trigger(
                () -> closeEnough.getAsBoolean() || elevatorNotAboveFinalLineupLevel.getAsBoolean());
        return drivetrain
                .newDriveToPoseAuton(() -> targetPose, CoralOffset, Seconds.of(5), maxAcceleration, distanceToTarget,
                        stoppingDistance)
                .get().raceWith(
                        elevator.newPidControlCommand(prescoreLevel).until(close)
                                .andThen(elevator.newPidControlCommand(level)
                                        .until(() -> elevator.getElevatorPosition().in(Meters) > Math.min(
                                                level.in(Meters), ElevatorSystem.Position.L3.position().in(Meters)))
                                        .onlyWhile(close))
                                .andThen(Commands.idle()))
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))
                        .andThen(shooter.getShootCommand(scoringLevel)).until(shooter.isEmpty()));
    }

    public static Command createNewTeleopCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance prescoreLevel = Meters.of(0.6);
        Distance finalLineupLevel = Meters.of(Math.min(1.2, level.in(Meters)));
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        double maxAcceleration = 4.3;
        BooleanSupplier close = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.25); // &&
                                                                             // Math.abs(drivetrain.getChassisSpeeds().vxMetersPerSecond)
                                                                             // < 1.0;
        BooleanSupplier notCloseAndSlow = () -> !close.getAsBoolean();
        BooleanSupplier closeEnough = () -> (targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation()) < 0.5);// &&
                                                                           // Math.abs(drivetrain.getChassisSpeeds().vxMetersPerSecond)
                                                                           // < 0.75;
        BooleanSupplier elevatorNotAboveFinalLineupLevel = () -> elevator.getElevatorPosition()
                .in(Meters) < finalLineupLevel.in(Meters);
        BooleanSupplier robotHasCoral = () -> shooter.getCoralState() == ShooterSystem.CoralState.FULLY_CAPTURED;
        Supplier<Translation2d> vectorToTarget = () -> targetPose.getTranslation()
                .minus(drivetrain.getPose().getTranslation());
        DoubleSupplier relativeAngleTowardsTarget = () -> Math
                .acos(((vectorToTarget.get().getX() * drivetrain.getFieldVelocity().vxMetersPerSecond)
                        + (vectorToTarget.get().getY() * drivetrain.getFieldVelocity().vyMetersPerSecond))
                        / (vectorToTarget.get().getNorm()
                                * Math.pow(Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                                        + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2), 0.5)));
        DoubleSupplier stoppingDistance = () -> ((Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2)
                + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2))
                * Math.pow(Math.cos(relativeAngleTowardsTarget.getAsDouble()), 2)) / (2 * maxAcceleration);
        DoubleSupplier distanceToTarget = () -> targetPose.getTranslation()
                .getDistance(drivetrain.getPose().getTranslation());
        Trigger closeEnoughOrNotAboveFinalLineupLevel = new Trigger(
                () -> closeEnough.getAsBoolean() || elevatorNotAboveFinalLineupLevel.getAsBoolean());
        return (drivetrain
                .newDriveToPoseAuton(() -> targetPose, CoralOffset, Seconds.of(5), maxAcceleration, distanceToTarget,
                        stoppingDistance)
                .get().raceWith(
                        elevator.newPidControlCommand(prescoreLevel).until(close)
                                .andThen(elevator.newPidControlCommand(level).until(
                                        () -> elevator.getElevatorPosition().in(Meters) > finalLineupLevel.in(Meters))
                                        .onlyWhile(close))
                                .andThen(Commands.idle()))
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))))
                .onlyIf(robotHasCoral);
    }

    public static Command createLoadingSequence(Pose2d targetPose) {
        Distance level = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);

        return drivetrain.driveToPosePrecise(targetPose, StationOffset).get()
                .alongWith(elevator.pidControlCommand(level));
    }

    public static Command createAutoLoadingSequence(Pose2d targetPose, double feedtimeout) {
        Distance l3Level = elevator.selectElevatorLevel(() -> ScoringLevel.L3);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        PathConstraints constraints = new PathConstraints(4.3,
                8.0,
                6.0,
                6.0);
        return Commands.sequence(
                elevator.pidControlCommand(intakeLevel).until(() -> elevator.isAtLocation(intakeLevel))
                        .andThen(elevator.elevatorPositionZeroSequence())
                        .andThen(elevator.holdElevatorDown()))
                .alongWith(
                        Commands.idle()
                                .until(() -> elevator.getElevatorPosition().in(Meters) < l3Level.in(Meters) + 0.8)
                                .andThen(
                                        Commands.parallel(
                                                drivetrain
                                                        .driveToPoseAuton(() -> targetPose, StationOffset,
                                                                Seconds.of(3), constraints)
                                                        .get(),
                                                shooter.intakeCoral()))
                                .until(shooter.coralCapturedOrAligned()))
                .until(shooter.coralCapturedOrAligned());
    }

    public static Command createAutoLoadingSequence(Pose2d targetPose) {
        return createAutoLoadingSequence(targetPose, 15.0);
    }

    public static Command driveXMetersQuest(Distance distance) {
        return drivetrain.driveToPose(
                drivetrain.getPose().transformBy(new Transform2d(distance.in(Meters), 0.0, drivetrain.getHeading())));
    }
}
