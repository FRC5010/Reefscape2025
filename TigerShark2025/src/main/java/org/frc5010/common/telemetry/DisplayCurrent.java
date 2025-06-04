package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Amps;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;

/** Add a current to the dashboard */
public class DisplayCurrent {
  /** Length value */
  MutCurrent current_;
  /** Length unit */
  protected final CurrentUnit unit_;
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
   * Add a current to the dashboard
   *
   * @param unit       - current unit
   * @param unitLength - current in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   */
  public DisplayCurrent(
      final double current, final CurrentUnit unit, final String name, final String table) {
    this(current, unit, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a current to the dashboard
   *
   * @param unit       - current unit
   * @param unitLength - current in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   * @param debug      - debug mode
   */
  public DisplayCurrent(
      final double current, final CurrentUnit unit, final String name, final String table, LogLevel logLevel) {
    current_ = new MutCurrent(current, unit.getBaseUnit().convertFrom(current, unit), unit);
    unit_ = unit;
    name_ = String.format("%s (%s)", name, unit_.symbol());
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.isAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
      publisher_ = topic_.publish();
      init(logLevel);
    }
  }

  // Constructor
  /**
   * Add a current to the dashboard
   *
   * @param unit  - current with units
   * @param name  - name of the variable
   * @param table - name of the table
   */
  public DisplayCurrent(final Current current, final String name, final String table) {
    this(current, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a current to the dashboard
   *
   * @param unit  - current with units
   * @param name  - name of the variable
   * @param table - name of the table
   * @param debug - debug mode
   */
  public DisplayCurrent(
      final Current current, final String name, final String table, LogLevel logLevel) {
    current_ = current.mutableCopy();
    unit_ = current.unit();
    name_ = String.format("%s (%s)", name, unit_.symbol());
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.isAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
      publisher_ = topic_.publish();
      init(logLevel);
    }
  }

  protected void init(LogLevel logLevel) {
    if (LogLevel.CONFIG == logLevel) {
      if (isDisplayed_) topic_.setPersistent(true);
      if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG)) {
        subscriber_ = topic_.subscribe(current_.in(unit_));
        listenerHandle_ = NetworkTableInstance.getDefault()
            .addListener(
                subscriber_,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                  setCurrent(event.valueData.value.getDouble(), unit_, false);
                });
        setCurrent(subscriber_.get(), unit_, false);
      }
    }
    publisher_.setDefault(current_.in(unit_));
  }

  // Setters
  /**
   * Sets the current
   *
   * @param unit       - current unit
   * @param unitLength - current in that unit
   */
  public void setCurrent(final double current, final CurrentUnit unit) {
    setCurrent(current, unit, true);
  }

  /**
   * Sets the current
   *
   * @param unit    - current unit
   * @param current - current in that unit
   * @param publish - publish the value
   */
  public void setCurrent(final double current, final CurrentUnit unit, final boolean publish) {
    setCurrent(unit.of(current), publish);
  }

  /**
   * Sets the current using a Current object and publishes the value.
   *
   * @param current - the Current object representing the current to set
   */
  public void setCurrent(final Current current) {
    setCurrent(current, true);
  }

  /**
   * Sets the current using a Current object
   *
   * @param current - the Current object representing the current to set
   * @param publish - whether or not to publish the value
   */
  public void setCurrent(final Current current, final boolean publish) {
    current_.mut_setBaseUnitMagnitude(current.baseUnitMagnitude());
    publish(publish);
  }

  /**
   * Sets the current using a DisplayCurrent object
   *
   * @param current the DisplayCurrent object to get the current from
   */
  public void setCurrent(final DisplayCurrent current) {
    setCurrent(current.current_, true);
  }

  /**
   * Publishes the Current to the network table if the publish flag is
   * true.
   *
   * @param publish - flag indicating whether to publish the current
   */
  protected void publish(boolean publish) {
    if (publish && isDisplayed_) {
      publisher_.set(current_.in(unit_));
    }
  }

  /**
   * Gets the Current
   *
   * @return the Current
   */
  public Current getCurrent() {
    return current_;
  }

  /**
   * Gets the current current in Amps
   *
   * @return the current current in Amps
   */
  public double getCurrentInAmps() {
    return current_.in(Amps);
  }
}
