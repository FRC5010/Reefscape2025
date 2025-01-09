package org.frc5010.common.telemetry;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add a float to the dashboard */
public class DisplayFloat {
  // Variables
  /** The value */
  protected float value_;
  /** The name */
  protected final String name_;
  /** The table */
  protected final String table_;
  /** The topic */
  protected FloatTopic topic_;
  /** The publisher */
  protected FloatPublisher publisher_;
  /** The subscriber */
  protected FloatSubscriber subscriber_;
  /** The listener handle */
  protected int listenerHandle_;
  /** Display mode */
  protected final boolean isDisplayed_;

  // Constructor
  /**
   * Add a float to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name of the variable
   * @param table        the name of the table
   */
  public DisplayFloat(final float defaultValue, final String name, final String table) {
    this(defaultValue, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a float to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name of the variable
   * @param table        the name of the table
   * @param debug        the debug mode
   */
  public DisplayFloat(final float defaultValue, final String name, final String table, final LogLevel logLevel) {
    value_ = defaultValue;
    name_ = name;
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.robotIsAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getFloatTopic(name_);
      publisher_ = topic_.publish();
      publisher_.setDefault(value_);
    }
    if (DisplayValuesHelper.robotIsAtLogLevel(LogLevel.CONFIG)) {
      topic_.setPersistent(true);
      subscriber_ = topic_.subscribe(value_);
      listenerHandle_ = NetworkTableInstance.getDefault()
          .addListener(
              subscriber_,
              EnumSet.of(NetworkTableEvent.Kind.kValueAll),
              event -> {
                setValue(event.valueData.value.getFloat(), false);
              });
    }
  }

  // Getters
  /**
   * Get the value
   *
   * @return the value
   */
  public synchronized float getValue() {
    return value_;
  }

  // Setters
  /**
   * Set the value
   *
   * @param value the value to set
   */
  public synchronized void setValue(final float value) {
    setValue(value, true);
  }

  /**
   * Set the value
   *
   * @param value   the value
   * @param publish - whether or not to publish
   */
  public synchronized void setValue(final float value, final boolean publish) {
    value_ = value;
    if (publish && isDisplayed_) {
      publisher_.set(value_);
    }
  }
}
