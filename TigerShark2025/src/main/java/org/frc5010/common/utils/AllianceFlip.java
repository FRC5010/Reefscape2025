// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.utils;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.frc5010.common.constants.FieldDimensions;
import org.frc5010.common.mechanisms.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AllianceFlip {
    static BooleanSupplier flipPose = () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        return false;
    };
    private static Boolean xFlip = true;
    private static Boolean yFlip = true;
    private static Distance fieldWidth;
    private static Distance fieldLength;

    private static Distance flipX(Distance x) {
            return fieldLength.minus(x);
    }

    private static Distance flipY(Distance y) {
        return fieldWidth.minus(y);
    }

    private static Rotation2d flipRotation(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.fromDegrees(180));
    }



    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(
            xFlip ? flipX(pose.getMeasureX()) : pose.getMeasureX(),
            yFlip ? flipY(pose.getMeasureY()) : pose.getMeasureY(),
            flipRotation(pose.getRotation())
        );
    }

    public static Pose2d apply(Pose2d pose) {
        return flipPose.getAsBoolean() ? flipPose(pose) : pose;
    }

    public static void configure(Distance width, Distance length, BooleanSupplier flip, Boolean flipX, Boolean flipY) {
        setFieldDimensions(width, length);
        setFlipSupplier(flip);
        setFlipX(flipX);
        setFlipY(flipY);
    }

    public static void configure(FieldDimensions field, BooleanSupplier flip) {
        configure(field.fieldWidth, field.fieldLength, flip, field.xFlip, field.yFlip);
    }

    public static void configure(FieldDimensions field) {
        setFieldDimensions(field.fieldWidth, field.fieldLength);
        setFlipX(field.xFlip);
        setFlipY(field.yFlip);
    }

    public static void setFieldDimensions(Distance width, Distance length) {
        fieldWidth = width;
        fieldLength = length;
    }

    public static void setFlipSupplier(BooleanSupplier flip) {
        flipPose = flip;
    }

    public static void setFlipX(Boolean flip) {
        xFlip = flip;
    }

    public static void setFlipY(Boolean flip) {
        yFlip = flip;
    }


}
