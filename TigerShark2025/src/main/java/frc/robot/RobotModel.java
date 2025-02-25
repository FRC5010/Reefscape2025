// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.ElevatorSystem;

/** Add your docs here. */
public class RobotModel {
    private Supplier<Translation3d> centerOfMassSupplier;
    private Supplier<Translation3d> centerOfMassVelocitySupplier;
    private Supplier<Translation3d> centerOfMassAccelerationSupplier;
    private Supplier<Double> dampeningFactor = () -> 1.0;

    private SwerveConstants drivetrainConstants = new SwerveConstants(Meters.zero(), Meters.zero());
    private LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.8);

    public RobotModel() {
    }

    public Translation3d getCenterOfMass() {
        return centerOfMassSupplier.get();
    }

    public Translation3d getCenterOfMassVelocity() {
        return centerOfMassVelocitySupplier.get();
    }

    public Translation3d getCenterOfMassAcceleration() {
        return centerOfMassAccelerationSupplier.get();
    }

    public void setupCenterOfMass(Supplier<Translation3d> centerOfMassSupplier,
            Supplier<Translation3d> centerOfMassVelocitySupplier,
            Supplier<Translation3d> centerOfMassAccelerationSupplier) {
        this.centerOfMassSupplier = centerOfMassSupplier;
        this.centerOfMassVelocitySupplier = centerOfMassVelocitySupplier;
        this.centerOfMassAccelerationSupplier = centerOfMassAccelerationSupplier;
    }

    public void setAccelerationDampening(Supplier<Double> dampeningFactor) {
        this.dampeningFactor = dampeningFactor;
    }

    public void setupDrivetrainConstants(SwerveConstants drivetrainConstants) {
        this.drivetrainConstants = drivetrainConstants;
    }

    public double getAccelerationDampener() {
        return dampeningFactor.get();
    }

    private LinearAcceleration getMaxDirectionalAcceleration(Distance torqueLength) {
        Translation3d centerOfMass = getCenterOfMass();
        double gravitationalTorque = torqueLength.in(Meters)
                * (gravity.in(MetersPerSecondPerSecond) + getCenterOfMassAcceleration().getZ());
        return MetersPerSecondPerSecond.of(gravitationalTorque / centerOfMass.getZ() * getAccelerationDampener());
    }

    public double getMaxForwardAcceleration() {
        return getMaxDirectionalAcceleration(
                drivetrainConstants.getWheelBase().div(2).plus(getCenterOfMass().getMeasureY()))
                .in(MetersPerSecondPerSecond) * getAccelerationDampener();
    }

    public double getMaxBackwardAcceleration() {
        return -getMaxDirectionalAcceleration(
                drivetrainConstants.getWheelBase().div(2).minus(getCenterOfMass().getMeasureY()))
                .in(MetersPerSecondPerSecond) * getAccelerationDampener();
    }

    public double getMaxRightAcceleration() {
        return getMaxDirectionalAcceleration(
                drivetrainConstants.getTrackWidth().div(2).plus(getCenterOfMass().getMeasureX()))
                .in(MetersPerSecondPerSecond) * getAccelerationDampener();
    }

    public double getMaxLeftAcceleration() {
        return -getMaxDirectionalAcceleration(
                drivetrainConstants.getTrackWidth().div(2).minus(getCenterOfMass().getMeasureX()))
                .in(MetersPerSecondPerSecond) * getAccelerationDampener();
    }

    public static boolean circularObstacleWillBeAvoided(Pose2d obstaclePosition, Supplier<Pose2d> robotPose, Pose2d targetPose,
            Pose2d[] unavoidableVertices, double obstacleRadius, double robotRadius, double maxRobotDimensionDeviation,
            double maxObstacleDimensionDeviation, int resolution) {
        Transform2d translationVector = new Transform2d(robotPose.get(), targetPose);
        Transform2d iterativeTranslation = new Transform2d();
        Pose2d newRobotPose = new Pose2d();
        double distance = 0.0, allowableDistance = obstacleRadius + robotRadius;
        for (int i = 0; i < resolution; i++) {
            iterativeTranslation = translationVector.times((double) i / resolution);
            newRobotPose = robotPose.get().plus(iterativeTranslation);
            distance = obstaclePosition.minus(newRobotPose).getTranslation().getNorm();
            if (distance < allowableDistance) {
                if (distance < allowableDistance + maxObstacleDimensionDeviation + maxRobotDimensionDeviation
                        || isToCloseToVertices(newRobotPose, unavoidableVertices, robotRadius)) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean isToCloseToVertices(Pose2d robotPose, Pose2d[] vertices, double robotRadius) {
        for (Pose2d vertice : vertices) {
            if (vertice.minus(robotPose).getTranslation().getNorm() < robotRadius) {
                return true;
            }
        }
        return false;
    }
}