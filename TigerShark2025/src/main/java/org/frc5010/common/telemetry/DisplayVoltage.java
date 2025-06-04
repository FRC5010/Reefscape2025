package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Volts;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** Add a voltage to the dashboard */
public class DisplayVoltage {
  /** Length value */
  MutVoltage voltage_;
  /** Length unit */
  protected final VoltageUnit unit_;
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
   * Add a voltage to the dashboard
   *
   * @param unit       - voltage unit
   * @param unitLength - voltage in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   */
  public DisplayVoltage(
      final double voltage, final VoltageUnit unit, final String name, final String table) {
    this(voltage, unit, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a voltage to the dashboard
   *
   * @param unit       - voltage unit
   * @param unitLength - voltage in that unit
   * @param name       - name of the variable
   * @param table      - name of the table
   * @param debug      - debug mode
   */
  public DisplayVoltage(
      final double voltage, final VoltageUnit unit, final String name, final String table, LogLevel logLevel) {
    voltage_ = new MutVoltage(voltage, unit.getBaseUnit().convertFrom(voltage, unit), unit);
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
   * Add a voltage to the dashboard
   *
   * @param unit  - voltage with units
   * @param name  - name of the variable
   * @param table - name of the table
   */
  public DisplayVoltage(final Voltage voltage, final String name, final String table) {
    this(voltage, name, table, LogLevel.COMPETITION);
  }

  /**
   * Add a voltage to the dashboard
   *
   * @param unit  - voltage with units
   * @param name  - name of the variable
   * @param table - name of the table
   * @param debug - debug mode
   */
  public DisplayVoltage(
      final Voltage voltage, final String name, final String table, LogLevel logLevel) {
    voltage_ = voltage.mutableCopy();
    unit_ = voltage.unit();
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
        subscriber_ = topic_.subscribe(voltage_.in(unit_));
        listenerHandle_ = NetworkTableInstance.getDefault()
            .addListener(
                subscriber_,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                  setVoltage(event.valueData.value.getDouble(), unit_, false);
                });
        setVoltage(subscriber_.get(), unit_, false);
      }
    }
    publisher_.setDefault(voltage_.in(unit_));
  }

  // Setters
  /**
   * Sets the voltage
   *
   * @param unit       - voltage unit
   * @param unitLength - voltage in that unit
   */
  public void setVoltage(final double voltage, final VoltageUnit unit) {
    setVoltage(voltage, unit, true);
  }

  /**
   * Sets the voltage
   *
   * @param unit       - voltage unit
   * @param unitLength - voltage in that unit
   * @param publish    - publish the value
   */
  public void setVoltage(final double voltage, final VoltageUnit unit, final boolean publish) {
    setVoltage(unit.of(voltage), publish);
  }

  /**
   * Sets the voltage using a Voltage object and publishes the value.
   *
   * @param voltage - the Voltage object representing the voltage to set
   */
  public void setVoltage(final Voltage voltage) {
    setVoltage(voltage, true);
  }

  /**
   * Sets the voltage using a Voltage object
   *
   * @param voltage - the Voltage object representing the voltage to set
   * @param publish - whether or not to publish the value
   */
  public void setVoltage(final Voltage voltage, final boolean publish) {
    voltage_.mut_setBaseUnitMagnitude(voltage.baseUnitMagnitude());
    publish(publish);
  }

  /**
   * Sets the voltage using a DisplayLength object
   *
   * @param voltage the DisplayLength object to get the voltage from
   */
  public void setVoltage(final DisplayVoltage voltage) {
    setVoltage(voltage.voltage_, true);
  }

  /**
   * Publishes the current voltage to the network table if the publish flag is
   * true.
   *
   * @param publish - flag indicating whether to publish the voltage
   */
  protected void publish(boolean publish) {
    if (publish && isDisplayed_) {
      publisher_.set(voltage_.in(unit_));
    }
  }

  /**
   * Gets the current voltage
   *
   * @return the current voltage
   */
  public Voltage getVoltage() {
    return voltage_;
  }

  public double getVoltageInVolts() {
    return voltage_.in(Volts);
  }
}
