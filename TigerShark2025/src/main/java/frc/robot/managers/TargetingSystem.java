// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


    public static void setupParameters(YAGSLSwerveDrivetrain drivetrain, ShooterSystem shooter, ElevatorSystem elevator,
            AlgaeArm arm) {
        TargetingSystem.drivetrain = drivetrain;
        TargetingSystem.shooter = shooter;
        TargetingSystem.elevator = elevator;
        TargetingSystem.arm = arm;

        


        

    }

    private static double getDrivetrainSpeed() {
        return new Translation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond,   
                                  drivetrain.getChassisSpeeds().vyMetersPerSecond).getNorm();
    }

    private static Supplier<Distance> getMaxHeightSupplier(Pose2d targetPose) {
        return () -> {
            if (targetPose.getTranslation().getDistance(drivetrain.getPose().getTranslation()) < 1.0 || getDrivetrainSpeed() > 0.5) {
                return MAX_NO_TIPPY_HEIGHT;
            } else {
                return Meters.of(2);
            }
        };
    }

    public static Command createCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);

        return drivetrain.driveToPosePrecise(targetPose).get()
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level)));
    }

    public static Command createAutoCoralScoringSequence(Pose2d targetPose, ScoringLevel scoringLevel) {
        Distance level = elevator.selectElevatorLevel(() -> scoringLevel);
        Distance intakeLevel = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);
        return drivetrain.driveToPosePrecise(targetPose).get()
                .andThen(elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level))
                        .andThen(shooter.runMotors(() -> 1.0).until(shooter.isEmpty()))).andThen(elevator.pidControlCommand(intakeLevel).until(() -> elevator.isAtLocation(intakeLevel)));
    }

    public static Command createLoadingSequence(Pose2d targetPose) {
        Distance level = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);

        return drivetrain.driveToPosePrecise(targetPose).get()
                .alongWith(Commands.runOnce(() -> SmartDashboard.putNumber("Elevator Level:", level.in(Meters))),
                        elevator.pidControlCommand(level));
    }

    public static Command createAutoLoadingSequence(Pose2d targetPose) {
        Distance level = elevator.selectElevatorLevel(() -> ScoringLevel.INTAKE);

        // return drivetrain.driveToPosePrecise(targetPose).get()
        //         .alongWith(Commands.runOnce(() -> SmartDashboard.putNumber("Elevator Level:", level.in(Meters))),
        //                 elevator.pidControlCommand(level));
        return elevator.pidControlCommand(level).until(() -> elevator.isAtLocation(level)).andThen(drivetrain.driveToPosePrecise(targetPose).get().alongWith(shooter.runMotors(() -> 1.0).until(shooter.isFullyCaptured()).andThen(Commands.runOnce(() -> {
            shooter.shooterLeftSpeed(0);
            shooter.shooterRightSpeed(0);
        }, shooter))));
    }
}
