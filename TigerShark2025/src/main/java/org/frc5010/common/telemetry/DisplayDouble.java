package org.frc5010.common.telemetry;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add a double to the dashboard */
public class DisplayDouble {
  // Variables
  /** The value */
  protected double value_;
  /** The name */
  protected final String name_;
  /** The table */
  protected final String table_;
  /** The topic */
  protected DoubleTopic topic_;
  /** The publisher */
  protected DoublePublisher publisher_;
  /** The subscriber */
  protected DoubleSubscriber subscriber_;
  /** The listener handle */
  protected int listenerHandle_;
  /** Display mode */
  protected final boolean isDisplayed_;

  // Constructor
  /**
   * Add a double to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name
   * @param table        the table
   */
  public DisplayDouble(final double defaultValue, final String name, final String table) {
    this(defaultValue, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a double to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name
   * @param table        the table
   * @param debug        debug
   */
  public DisplayDouble(final double defaultValue, final String name, final String table, final LogLevel logLevel) {
    value_ = defaultValue;
    name_ = name;
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.robotIsAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
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
                setValue(event.valueData.value.getDouble(), false);
              });
    }
  }

  // Getters
  /**
   * Get the value
   *
   * @return the value
   */
  public synchronized double getValue() {
    return value_;
  }

  // Setters
  /**
   * Set the value
   *
   * @param value the value
   */
  public synchronized void setValue(final double value) {
    setValue(value, true);
  }

  /**
   * Set the value
   *
   * @param value   the value to set
   * @param publish whether or not to publish the value
   */
  public synchronized void setValue(final double value, final boolean publish) {
    value_ = value;
    if (publish && isDisplayed_) {
      publisher_.set(value_);
    }
  }
}
