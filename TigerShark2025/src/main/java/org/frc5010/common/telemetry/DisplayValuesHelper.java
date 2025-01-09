// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.arch.WpiHelperInterface;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Registers values with the network tables */
public class DisplayValuesHelper implements WpiHelperInterface {
    protected ShuffleboardTab tab;
    protected ShuffleboardLayout layout;
    protected String tabName = "frc";
    protected String layoutName = "5010";
    protected int column = 0;
    protected boolean isDisplayed;

    public DisplayValuesHelper(String tab, String table) {
        this(tab, table, false, 0);
    }

    public DisplayValuesHelper(String tab, String table, boolean isDisplayed) {
        this(tab, table, isDisplayed, 0);
    }

    public DisplayValuesHelper(String tab, String table, boolean isDisplayed, int startingColumn) {
        column = startingColumn;
        this.isDisplayed = isDisplayed;
        tabName = tab;
        layoutName = table;
        if (!isDisplayed) return;

        makeDisplayed();
    }

    public void makeDisplayed() {
        isDisplayed = true;
        this.tab = Shuffleboard.getTab(tabName);
        this.layout = this.tab.getLayout(layoutName, BuiltInLayouts.kList).withSize(2, 4).withPosition(column, 0);
    }
    /**
     * Advances the column number for the next value to be placed in.
     *
     * @param name the name of the next column
     */
    public void nextColumn(String name) {
        layoutName = name;
        if (!isDisplayed) return;
        column += 2;
        layout = tab.getLayout(name, BuiltInLayouts.kList).withSize(2, 4).withPosition(column, 0);
    }

    /**
     * Constructs the NetworkTable folder path for the current Shuffleboard layout.
     *
     * @return a String representing the path in the format
     *         "Shuffleboard/<tabTitle>/<layoutTitle>"
     */
    private String getNtFolder() {
        if (!isDisplayed) return "SmartDashboard/" + tabName + "/" + layoutName;
        return "Shuffleboard/" + tabName + "/" + layoutName;
    }

    /**
     * Checks if the robot's current logging level is at or above the specified log
     * level.
     *
     * @param logLevel the log level to check against
     * @return true if the robot's current log level is at or above the specified
     *         level,
     *         false otherwise
     */
    public static boolean robotIsAtLogLevel(LogLevel logLevel) {
        switch (logLevel) {
            case DEBUG: {
                return GenericRobot.logLevel == LogLevel.DEBUG || GenericRobot.logLevel == LogLevel.INFO;
            }
            case INFO: {
                return GenericRobot.logLevel == LogLevel.INFO || GenericRobot.logLevel == LogLevel.DEBUG;
            }
            case CONFIG: {
                return GenericRobot.logLevel == LogLevel.CONFIG || GenericRobot.logLevel == LogLevel.DEBUG;
            }
            case COMPETITION: {
                return true;
            }
            default: {
                return true;
            }
        }
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers
     * it with NetworkTables.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeDisplayAngle(String name) {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder());
        return angle;
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers
     * it with NetworkTables, at the INFO logging level.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeInfoAngle(String name) {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder(), LogLevel.INFO);
        return angle;
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers
     * it with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeConfigAngle(String name) {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder(), LogLevel.CONFIG);
        return angle;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and
     * registers it with NetworkTables.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeDisplayLength(String name) {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder());
        return length;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and
     * registers it with NetworkTables, at the INFO logging level.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeInfoLength(String name) {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder(), LogLevel.INFO);
        return length;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and
     * registers it with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeConfigLength(String name) {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder(), LogLevel.CONFIG);
        return length;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers
     * it with NetworkTables.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeDisplayTime(String name) {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder());
        return time;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers
     * it with NetworkTables, at the INFO logging level.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeInfoTime(String name) {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder(), LogLevel.INFO);
        return time;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers
     * it with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeConfigTime(String name) {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder(), LogLevel.CONFIG);
        return time;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and
     * registers
     * it with NetworkTables.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeDisplayVoltage(String name) {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder());
        return voltage;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and
     * registers it with NetworkTables, at the INFO logging level.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeInfoVoltage(String name) {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder(), LogLevel.INFO);
        return voltage;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and
     * registers it with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeConfigVoltage(String name) {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder(), LogLevel.CONFIG);
        return voltage;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it with
     * NetworkTables.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeDisplayBoolean(String name) {
        DisplayBoolean booleanValue = new DisplayBoolean(false, name, getNtFolder());
        return booleanValue;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it
     * with NetworkTables at the INFO logging level.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeInfoBoolean(String name) {
        DisplayBoolean booleanValue = new DisplayBoolean(false, name, getNtFolder(), LogLevel.INFO);
        return booleanValue;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it
     * with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeConfigBoolean(String name) {
        DisplayBoolean booleanValue = new DisplayBoolean(false, name, getNtFolder(), LogLevel.CONFIG);
        return booleanValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with
     * NetworkTables.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeDisplayDouble(String name) {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder());
        return doubleValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with
     * NetworkTables at the INFO logging level.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeInfoDouble(String name) {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder(), LogLevel.INFO);
        return doubleValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with
     * NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeConfigDouble(String name) {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder(), LogLevel.CONFIG);
        return doubleValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers
     * it with NetworkTables.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeDisplayString(String name) {
        DisplayString stringValue = new DisplayString("", name, getNtFolder());
        return stringValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers it
     * with NetworkTables at the INFO logging level.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeInfoString(String name) {
        DisplayString stringValue = new DisplayString("", name, getNtFolder(), LogLevel.INFO);
        return stringValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers it
     * with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeConfigString(String name) {
        DisplayString stringValue = new DisplayString("", name, getNtFolder(), LogLevel.CONFIG);
        return stringValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers
     * it with NetworkTables.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeDisplayLong(String name) {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder());
        return longValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers
     * it with NetworkTables at the INFO logging level.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeInfoLong(String name) {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder(), LogLevel.INFO);
        return longValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers it
     * with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeConfigLong(String name) {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder(), LogLevel.CONFIG);
        return longValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers
     * it with NetworkTables.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeDisplayFloat(String name) {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder());
        return floatValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers it
     * with NetworkTables at the INFO logging level.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeInfoFloat(String name) {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder(), LogLevel.INFO);
        return floatValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers it
     * with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeConfigFloat(String name) {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder(), LogLevel.CONFIG);
        return floatValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers
     * it with NetworkTables.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeDisplayCurrent(String name) {
        DisplayCurrent currentValue = new DisplayCurrent(0, Amps, name, getNtFolder());
        return currentValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers
     * it with NetworkTables at the INFO logging level.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeInfoCurrent(String name) {
        DisplayCurrent currentValue = new DisplayCurrent(0, Amps, name, getNtFolder(), LogLevel.INFO);
        return currentValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers it
     * with NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeConfigCurrent(String name) {
        DisplayCurrent currentValue = new DisplayCurrent(0, Amps, name, getNtFolder(), LogLevel.CONFIG);
        return currentValue;
    }
}
