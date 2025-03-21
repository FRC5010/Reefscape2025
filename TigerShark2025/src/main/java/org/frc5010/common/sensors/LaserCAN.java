// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import au.grapplerobotics.LaserCan;

/** Add your docs here. */
public class LaserCAN {
    private LaserCan laserCan;

    public LaserCAN(int canID) {
        laserCan = new LaserCan(canID);
    }

    public double getDistance() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        //  From Example Code:
        //    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        //     System.out.println("The target is " + measurement.distance_mm + "mm away!");
        // } else {
        //     System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
        //     // You can still use distance_mm in here, if you're ok tolerating a clamped
        //     // value or an unreliable measurement.
        // }
        if (null != measurement) {
            return measurement.distance_mm;
        }
        return -1.0;
        
    }
}
