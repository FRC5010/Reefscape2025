// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import org.frc5010.common.constants.SwerveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RobotModel {
    private Supplier<Translation3d> centerOfMassSupplier;
    private Supplier<Translation3d> centerOfMassVelocitySupplier;
    private Supplier<Translation3d> centerOfMassAccelerationSupplier;
    private Supplier<Double> dampeningFactor = () -> 1.0;
    public static final Distance ROBOT_SIZE = Inches.of(34.75);
    public static final Distance HALF_ROBOT_SIZE = ROBOT_SIZE.div(2);

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

    public static boolean circularObstacleWillBeAvoided(Pose2d obstaclePosition, Supplier<Pose2d> robotPose,
            Pose2d targetPose,
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
                if (distance < allowableDistance + maxObstacleDimensionDeviation + maxRobotDimensionDeviation + 0.05
                        || isToCloseToVertices(newRobotPose, unavoidableVertices, robotRadius)) {
                    SmartDashboard.putBoolean("Has Linear Path", false);
                    return false;
                }
            }
        }
        SmartDashboard.putBoolean("Has Linear Path", true);
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

    private static boolean linesIntersect(Translation2d line1Start, Translation2d line1End, Translation2d line2Start,
            Translation2d line2End) {
        double x1 = line1Start.getX(), y1 = line1Start.getY(), x2 = line1End.getX(), y2 = line1End.getY();
        double x3 = line2Start.getX(), y3 = line2Start.getY(), x4 = line2End.getX(), y4 = line2End.getY();
        double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (denominator == 0) {
            return false;
        }
        double xNumerator = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4);
        double yNumerator = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4);
        double x = xNumerator / denominator;
        double y = yNumerator / denominator;
        return x >= Math.min(x1, x2) && x <= Math.max(x1, x2) && y >= Math.min(y1, y2) && y <= Math.max(y1, y2)
                && x >= Math.min(x3, x4) && x <= Math.max(x3, x4) && y >= Math.min(y3, y4) && y <= Math.max(y3, y4);
    }

    private static Translation2d[] constructCornerPoints(Pose2d pose, Distance width, Distance length) {
        Translation2d[] cornerPoints = new Translation2d[4];
        cornerPoints[0] = pose.transformBy(new Transform2d(width.div(2), length.div(2), new Rotation2d()))
                .getTranslation();
        cornerPoints[1] = pose.transformBy(new Transform2d(width.div(2), length.div(-2), new Rotation2d()))
                .getTranslation();
        cornerPoints[2] = pose.transformBy(new Transform2d(width.div(-2), length.div(-2), new Rotation2d()))
                .getTranslation();
        cornerPoints[3] = pose.transformBy(new Transform2d(width.div(-2), length.div(2), new Rotation2d()))
                .getTranslation();
        return cornerPoints;
    }

    private static Translation2d[][] optimizeEdgeChecks(Translation2d[] convexPolygon, Pose2d startPose, Pose2d endPose,
            Distance robotRadius) {

        // TODO: Don't return edges which are impossble to intersect with the robot
        Translation2d[][] edges = new Translation2d[convexPolygon.length][2];
        for (int i = 0; i < convexPolygon.length; i++) {
            edges[i][0] = convexPolygon[i];
            edges[i][1] = convexPolygon[(i + 1) % convexPolygon.length];
        }
        return edges;

    }

    public static boolean robotHasLinearPath(Pose2d robotPose, Pose2d targetPose, Translation2d[] convexPolygon,
            Distance robotWidth, Distance robotLength) {
        Translation2d[] robotCornerPoints = constructCornerPoints(robotPose, robotWidth, robotLength);
        Translation2d[] targetCornerPoints = constructCornerPoints(targetPose, robotWidth, robotLength);

        Translation2d[][] obstacleEdges = optimizeEdgeChecks(convexPolygon, robotPose, targetPose, robotWidth);

        for (Translation2d[] edge : obstacleEdges) {
            for (int i = 0; i < robotCornerPoints.length; i++) {
                if (linesIntersect(robotCornerPoints[i], targetCornerPoints[i], edge[0], edge[1])) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean robotHasLinearPath(Pose2d robotPose, Pose2d targetPose, Pose2d[] convexPolygon,
            Distance robotWidth, Distance robotLength) {
        Translation2d[] vertices = new Translation2d[convexPolygon.length];
        for (int i = 0; i < convexPolygon.length; i++) {
            vertices[i] = convexPolygon[i].getTranslation();
        }
        return robotHasLinearPath(robotPose, targetPose, vertices, robotWidth, robotLength);
    }

    // TODO: Finish Function
    public static double percentWidthPoleIntersection(Supplier<Pose2d> robotPose, Distance robotRadius) {
        Pose2d frontLeftCorner = robotPose.get().transformBy(new Transform2d(new Translation2d(robotRadius, Meters.of(0.0)), new Rotation2d(Degrees.of(45)))); // Point 1
        Pose2d frontRightCorner = robotPose.get().transformBy(new Transform2d(new Translation2d(robotRadius, Meters.of(0.0)), new Rotation2d(Degrees.of(315)))); // Point 2
        Pose2d coralRayEnd = ReefscapeButtonBoard.getScoringPose().transformBy(new Transform2d(new Translation2d(Meters.of(-0.4), Inches.of(0.0)), new Rotation2d())); // Point 4
        Pose2d coralRayStart = coralRayEnd.transformBy(new Transform2d(new Translation2d(Meters.of(1.0), Meters.of(0.0)), new Rotation2d())); // Point 3
        double numerator = (coralRayStart.getX() - frontLeftCorner.getX()) * (coralRayEnd.getY() - coralRayStart.getY()) - (coralRayStart.getY() - frontLeftCorner.getY()) * (coralRayEnd.getX() - coralRayStart.getX());
        double denominator = (frontRightCorner.getX() - frontLeftCorner.getX()) * (coralRayEnd.getY() - coralRayStart.getY()) - (frontRightCorner.getY() - frontLeftCorner.getY()) * (coralRayEnd.getX() - coralRayStart.getX());
        SmartDashboard.putNumber("Percentage Intersect Numerator", numerator);
        SmartDashboard.putNumber("Percentage Intersect Denominator", denominator);
        SmartDashboard.putNumberArray("Percentage Intersect Front Left", new double[] {frontLeftCorner.getX(), frontLeftCorner.getY()});
        SmartDashboard.putNumberArray("Percentage Intersect Front Right", new double[] {frontRightCorner.getX(), frontRightCorner.getY()});
        SmartDashboard.putNumberArray("Percentage Intersect Coral Ray Start", new double[] {coralRayStart.getX(), coralRayStart.getY()});
        SmartDashboard.putNumberArray("Percentage Intersect Coral Ray End", new double[] {coralRayEnd.getX(), coralRayEnd.getY()});
        
        SmartDashboard.putNumber("Percentage Intersect Denominator", denominator);
        if (denominator == 0 || numerator / denominator > 1.0 || numerator / denominator < 0.0) {
            SmartDashboard.putNumber("Percentage Intersect", -1.0);
            return -1.0;
        }
        SmartDashboard.putNumber("Percentage Intersect", numerator / denominator);
        return numerator / denominator;
    }
}