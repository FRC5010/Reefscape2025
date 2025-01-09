package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Meters;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

/** Add a length to the dashboard */
public class DisplayLength {
  /** Length value */
  MutDistance length_;
  /** Length unit */
  protected final DistanceUnit unit_;
  /** The name of the variable */
  protected final String name_;
  /** The name of the table */
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
   * Add a length to the dashboard
   *
   * @param unit       - length unit
   * @param unitLength - length in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   */
  public DisplayLength(final double length, final DistanceUnit unit, final String name, final String table) {
    this(length, unit, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a length to the dashboard
   *
   * @param unit       - length unit
   * @param unitLength - length in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   * @param debug      - debug mode
   */
  public DisplayLength(
      final double length, final DistanceUnit unit, final String name, final String table, LogLevel logLevel) {
    length_ = new MutDistance(length, unit.getBaseUnit().convertFrom(length, unit), unit);
    unit_ = unit;
    name_ = String.format("%s (%s)", name, unit_.symbol());
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.robotIsAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
      publisher_ = topic_.publish();
      init();
    }
  }

  // Constructor
  /**
   * Add a length to the dashboard
   *
   * @param unit  - length with units
   * @param name  - name of the variable
   * @param table - name of the table
   */
  public DisplayLength(final Distance length, final String name, final String table) {
    this(length, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a length to the dashboard
   *
   * @param unit  - length with units
   * @param name  - name of the variable
   * @param table - name of the table
   */
  public DisplayLength(
      final Distance length, final String name, final String table, LogLevel logLevel) {
    length_ = length.mutableCopy();
    unit_ = length.unit();
    name_ = String.format("%s (%s)", name, unit_.symbol());
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.robotIsAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
      publisher_ = topic_.publish();
      init();
    }
  }

  protected void init() {
    publisher_.setDefault(length_.in(unit_));
    if (DisplayValuesHelper.robotIsAtLogLevel(LogLevel.CONFIG)) {
      topic_.setPersistent(true);
      subscriber_ = topic_.subscribe(length_.in(unit_));
      listenerHandle_ = NetworkTableInstance.getDefault()
          .addListener(
              subscriber_,
              EnumSet.of(NetworkTableEvent.Kind.kValueAll),
              event -> {
                setLength(event.valueData.value.getDouble(), unit_, false);
              });
    }
  }

  // Setters
  /**
   * Sets the length
   *
   * @param unit       - length unit
   * @param unitLength - length in that unit
   */
  public void setLength(final double length, final DistanceUnit unit) {
    setLength(length, unit, true);
  }

  /**
   * Sets the length
   *
   * @param unit       - length unit
   * @param unitLength - length in that unit
   * @param publish    - publish the value
   */
  public void setLength(final double length, final DistanceUnit unit, final boolean publish) {
    setLength(unit.of(length), publish);
  }

  /**
   * Sets the length using a Distance object and publishes the value.
   *
   * @param length - the Distance object representing the length to set
   */
  public void setLength(final Distance length) {
    setLength(length, true);
  }

  /**
   * Sets the length using a Distance object
   *
   * @param length  - the Distance object representing the length to set
   * @param publish - whether or not to publish the value
   */
  public void setLength(final Distance length, final boolean publish) {
    length_.mut_setBaseUnitMagnitude(length.baseUnitMagnitude());
    publish(publish);
  }

  /**
   * Sets the length using a DisplayLength object
   *
   * @param length the DisplayLength object to get the length from
   */
  public void setLength(final DisplayLength length) {
    setLength(length.length_, true);
  }

  /**
   * Publishes the current length to the network table if the publish flag is
   * true.
   *
   * @param publish - flag indicating whether to publish the length
   */
  protected void publish(boolean publish) {
    if (publish && isDisplayed_) {
      publisher_.set(length_.in(unit_));
    }
  }

  /**
   * Get the length
   *
   * @return the length
   */
  public Distance getLength() {
    return length_;
  }

  public double getLengthInMeters() {
    return length_.in(Meters);
  }
}
