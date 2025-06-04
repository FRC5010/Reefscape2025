package org.frc5010.common.telemetry;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;

/** Add a string to the dashboard */
public class DisplayString {
  // Variables
  /** The value */
  protected String value_;
  /** The name */
  protected final String name_;
  /** The name of the table */
  protected final String table_;
  /** The topic */
  protected StringTopic topic_;
  /** The publisher */
  protected StringPublisher publisher_;
  /** The subscriber */
  protected StringSubscriber subscriber_;
  /** The listener handle */
  protected int listenerHandle_;
  /** Display mode */
  protected final boolean isDisplayed_;

  // Constructor
  /**
   * Add a string to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name of the variable
   * @param table        the name of the table
   */
  public DisplayString(final String defaultValue, final String name, final String table) {
    this(defaultValue, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a string to the dashboard
   *
   * @param defaultValue the default value
   * @param name         the name of the variable
   * @param table        the name of the table
   * @param debug        the debug mode
   */
  public DisplayString(final String defaultValue, final String name, final String table, final LogLevel logLevel) {
    value_ = defaultValue;
    name_ = name;
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.isAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getStringTopic(name_);
      publisher_ = topic_.publish();
      if (LogLevel.CONFIG == logLevel) {
        if (isDisplayed_) topic_.setPersistent(true);
        if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG)) {
          subscriber_ = topic_.subscribe(value_);
          listenerHandle_ = NetworkTableInstance.getDefault()
              .addListener(
                  subscriber_,
                  EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                  event -> {
                    setValue(event.valueData.value.getString(), false);
                  });
          setValue(subscriber_.get(), false);
        }
        publisher_.setDefault(value_);
      }
    }
  }

  // Getters
  /**
   * Get the value
   *
   * @return the value
   */
  public synchronized String getValue() {
    return value_;
  }

  // Setters
  /**
   * Set the value
   *
   * @param value the value to set
   */
  public synchronized void setValue(final String value) {
    setValue(value, true);
  }

  /**
   * Set the value
   *
   * @param value   the value to set
   * @param publish whether or not to publish the value
   */
  public synchronized void setValue(final String value, final boolean publish) {
    value_ = value;
    if (publish && isDisplayed_) {
      publisher_.set(value_);
    }
  }
}
