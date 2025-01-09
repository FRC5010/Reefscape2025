// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation3d;
import swervelib.SwerveDrive;

/** An class for accessing the yagsl gyro */
public class YagslGyro implements GenericGyro {
    private SwerveDrive swerveDrive = YAGSLSwerveDrivetrain.getSwerveDrive();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    @Override
    public void reset() {
        swerveDrive.zeroGyro();
    }

    /**
     * Gets the angle of the gyro
     * 
     * @return the angle of the gyro in degrees
     */
    @Override
    public double getAngle() {
        return swerveDrive.getYaw().getDegrees();
    }

    /**
     * Gets the angle of the gyro on the x-axis (roll)
     * 
     * @return the angle of the gyro on the x-axis in degrees
     */
    @Override
    public double getAngleX() {
        return swerveDrive.getRoll().getDegrees();
    }

    /**
     * Gets the angle of the gyro on the y-axis (pitch)
     * 
     * @return the angle of the gyro on the y-axis in degrees
     */
    @Override
    public double getAngleY() {
        return swerveDrive.getPitch().getDegrees();
    }

    /**
     * Gets the angle of the gyro on the z-axis (yaw)
     * 
     * @return the angle of the gyro on the z-axis in degrees
     */
    @Override
    public double getAngleZ() {
        return swerveDrive.getYaw().getDegrees();
    }

    /**
     * Gets the rate of the gyro
     * 
     * @return the rate of the gyro in degrees per second
     */
    @Override
    public double getRate() {
        return swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);
    }

    /**
     * Sets the angle of the gyro to the given angle.
     * 
     * @param angle - the angle to set in degrees
     */
    @Override
    public void setAngle(double angle) {
        swerveDrive.getGyro().setOffset(new Rotation3d(0, 0, getAngleZ() -angle));
    }
}
