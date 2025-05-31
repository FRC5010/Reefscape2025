// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.auto;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import org.frc5010.common.arch.GenericCommandSequence;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** A class for creating complex autonomous sequences which include event-based triggers */
public class AutoSequence extends GenericCommandSequence {
    /** Autonomous event loop used to bind triggers which are only active while the autonomous sequence is running */
    EventLoop autonomousEventLoop = new EventLoop();


    /**
     * Returns a command that resets the odometry to the specified pose.
     * 
     * @param pose The pose to which to set the odometry.
     * @return The command that will reset the odometry.
     */
    public Command resetOdometry(Pose2d pose) {
        return AutoBuilder.resetOdom(pose);
    }

    /**
     * Returns the autonomous event loop used to bind triggers which are only active while the autonomous sequence is running.
     * @return The autonomous event loop.
     */
    public EventLoop getAutonomousEventLoop() {
        return autonomousEventLoop;
    }

    /**
     * Constructs a trigger that is only active during autonomous mode. Can be used for event-based autonomous sequences.
     * @param condition The condition that must be true for the trigger to be active.
     * @return The trigger that is only active during autonomous mode.
     */
    public Trigger autonTrigger(BooleanSupplier condition) {
        return new Trigger(getAutonomousEventLoop(), condition);
    }

    /**
     * Constructs an trigger which is active when the robot is within a rectangular region.
     * @param topLeft The top left corner of the rectangular region.
     * @param bottomRight The bottom right corner of the rectangular region.
     * @return The trigger which is active when the robot is within the rectangular region.
     */
    public Trigger robotWithinRectangularRegion(Pose2d topLeft, Pose2d bottomRight) {
        return autonTrigger(() -> {
            Pose2d robotPose = AutoBuilder.getCurrentPose();
            return robotPose.getX() >= topLeft.getX() && robotPose.getX() <= bottomRight.getX() && robotPose.getY() >= bottomRight.getY() && robotPose.getY() <= topLeft.getY();
        });
    }

    /**
     * Constructs a trigger which is active when the robot is within a circular region.
     * @param center The center of the circular region.
     * @param radius The radius of the circular region.
     * @return The trigger which is active when the robot is within the circular region.
     */
    public Trigger robotWithinCircularRegion(Pose2d center, Distance radius) {
        return autonTrigger(() -> {
            Pose2d robotPose = AutoBuilder.getCurrentPose();
            return robotPose.getTranslation().getDistance(center.getTranslation()) <= radius.in(Meters);
        }); 
    }

    /**
     * Constructs a trigger which is active when the robot is near a location.
     * @param location The Pose2d location at which the trigger should be active.
     * @param distance The distance from the location at which the trigger should be active.
     * @return The trigger which is active when the robot is near the location.
     */
    public Trigger robotNearLocation(Pose2d location, Distance distance) {
        return autonTrigger(() -> {
            Pose2d robotPose = AutoBuilder.getCurrentPose();
            return robotPose.getTranslation().getDistance(location.getTranslation()) <= distance.in(Meters);
        });
    }

    public int getCurrentCommandIndex() {
        return m_currentCommandIndex;
    }

    /**
     * Constructs a trigger which mirrors the state of an event marker. Used with the EventMarkerFlag command.
     */
    public Trigger EventMarker(String name) {
        return EventMarkerFlag.getTrigger(name, getAutonomousEventLoop());
    }

    @Override
    public void execute() {
        super.execute();
        autonomousEventLoop.poll();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        autonomousEventLoop.clear();
    }


    
}