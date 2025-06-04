package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Degrees;

import java.util.EnumSet;

import org.frc5010.common.arch.GenericRobot.LogLevel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

/** Add an angle to the dashboard */
public class DisplayAngle {
  /** Angle value */
  final MutAngle angle_;
  /** Angle unit */
  protected final AngleUnit unit_;
  /** Name of the angle value */
  protected final String name_;
  /** Name of the table to display the value */
  protected final String table_;
  /** Topic for the angle */
  protected DoubleTopic topic_;
  /** Publisher for the angle */
  protected DoublePublisher publisher_;
  /** Widget for the angle */
  protected SuppliedValueWidget<Double> component_;
  /** Subscriber for the angle */
  protected DoubleSubscriber subscriber_;
  /** Listener handle */
  protected int listenerHandle_;
  /** Display mode */
  protected final boolean isDisplayed_;

  /**
   * Constructor for an angle display
   *
   * @param angle - angle in that unit
   * @param unit  - angle unit
   * @param name  - name of the angle
   * @param table - name of the table
   */
  public DisplayAngle(
      final double angle, AngleUnit unit, final String name, final String table) {
    this(angle, unit, name, table, LogLevel.COMPETITION);
  }

  /**
   * Constructor for an angle display
   *
   * @param angle - angle in that unit
   * @param unit  - angle unit
   * @param name  - name of the angle
   * @param table - name of the table
   */
  public DisplayAngle(
      final double angle, AngleUnit unit, final String name, final String table, LogLevel logLevel) {
    angle_ = new MutAngle(angle, unit.getBaseUnit().convertFrom(angle, unit), unit);
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

  /**
   * Constructor for an angle display
   *
   * @param angle - angle with units
   * @param name  - name of the angle
   * @param table - name of the table
   */
  public DisplayAngle(
      final Angle angle, final String name, final String table) {
    this(angle, name, table, LogLevel.COMPETITION);
  }

  /**
   * Constructor for an angle display
   *
   * @param angle - angle with units
   * @param name  - name of the angle
   * @param table - name of the table
   * @param debug - whether or not to debug
   */
  public DisplayAngle(
      final Angle angle, final String name, final String table, LogLevel logLevel) {
    angle_ = angle.mutableCopy();
    unit_ = angle.unit();
    name_ = String.format("%s (%s)", name, unit_.symbol());
    table_ = table;
    isDisplayed_ = DisplayValuesHelper.isAtLogLevel(logLevel);
    if (isDisplayed_) {
      topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
      publisher_ = topic_.publish();
      init(logLevel);
    }
  }

  /**
   * Initializes the display by adding a listener to the subscriber and
   * setting the default value of the publisher
   */
  protected void init(LogLevel logLevel) {
    publisher_.setDefault(angle_.in(unit_));
    if (LogLevel.CONFIG == logLevel) {
      if (isDisplayed_) topic_.setPersistent(true);
      if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG)) {
        subscriber_ = topic_.subscribe(angle_.in(unit_));
        listenerHandle_ = NetworkTableInstance.getDefault()
            .addListener(
                subscriber_,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                  setAngle(event.valueData.value.getDouble(), unit_, false);
                });
        setAngle(subscriber_.get(), unit_, false);
      }
    }
  }

  // Setters
  /**
   * Sets the angle
   *
   * @param angle - angle in that unit
   * @param unit  - angle unit
   */
  public void setAngle(final double angle, final AngleUnit unit) {
    setAngle(angle, unit, true);
  }

  /**
   * Sets the angle
   *
   * @param angle   - angle in that unit
   * @param unit    - angle unit
   * @param publish - whether or not to publish
   */
  public void setAngle(final double angle, final AngleUnit unit, final boolean publish) {
    setAngle(unit.of(angle), publish);
  }

  /**
   * Sets the angle using base units
   *
   * @param unitAngle - angle with units
   * @param publish   - whether or not to publish
   */
  public void setAngle(final Angle unitAngle, final boolean publish) {
    angle_.mut_setBaseUnitMagnitude(unitAngle.baseUnitMagnitude());
    publish(publish);
  }

  /**
   * Sets the angle
   *
   * @param unitAngle - angle with units
   */
  public void setAngle(final Angle unitAngle) {
    setAngle(unitAngle, true);
  }

  /**
   * Sets the angle from another {@link DisplayAngle} object
   *
   * @param angle - another {@link DisplayAngle} object
   */
  public void setAngle(final DisplayAngle angle) {
    setAngle(angle.angle_, true);
  }

  /**
   * Publishes the current angle to the network table if the publish flag is true.
   *
   * @param publish - flag indicating whether to publish the angle
   */
  protected void publish(boolean publish) {
    if (publish && isDisplayed_) {
      publisher_.set(angle_.in(unit_));
    }
  }

  /**
   * Retrieves the current angle.
   *
   * @return the current angle as an {@link Angle} object
   */
  public Angle getAngle() {
    return angle_;
  }

  public double getAngleInDegrees() {
    return angle_.in(Degrees);
  }
}
