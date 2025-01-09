// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.auto;

import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericCommand;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class which acts as a communication layer between path event markers and robot code to allow for use of markers as triggers in autonomous sequences */
public class EventMarkerFlag extends GenericCommand {
    /** Map of marker names to their current state */
    private static Map<String, Boolean> flags = new HashMap<String, Boolean>();
    /** Name of the marker */
    private String name;

    public EventMarkerFlag(String name) {
        super("MarkerFlag " + name);
        this.name = name;
        flags.put(name, false);
    }

    /** Set the current state of the marker 
     * @param value The state to set the marker to */
    private void setFlag(boolean value) {
        flags.put(name, value);
    }

    /** On the creation of the marker, called by markers created on autonomous pathes, set the state to true */
    @Override
    public void init() {
        setFlag(true);
    }

    /** On the end of the marker command, called by markers created on autonomous pathes, set the state to false */
    @Override
    public void stop(boolean interrupted) {
        setFlag(false);
    }

    /** Get the current state of the marker 
     * @param name The name of the marker to get the state of 
     * @return The state of the marker */
    public static boolean get(String name) {
        return flags.get(name);
    }

    /** Constructs a trigger that is active when the marker is active 
     * @param name The name of the marker to create a trigger for 
     * @return The trigger that is active when the marker is active */
    public static Trigger getTrigger(String name) {
        return new Trigger(() -> EventMarkerFlag.get(name));
    }

    /** Constructs a trigger that is active when the marker is active and binds it to a specific event loop
     * @param name The name of the marker to create a trigger for
     * @param eventLoop The event loop to bind the trigger to
     * @return The trigger that is active when the marker is active */
    public static Trigger getTrigger(String name, EventLoop eventLoop) {
        return new Trigger(eventLoop, () -> EventMarkerFlag.get(name));
    }

}
