// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class Controller {

  private Joystick joystick;
  private boolean singleControllerMode;
  public JoystickButton A_BUTTON,
      B_BUTTON,
      X_BUTTON,
      Y_BUTTON,
      LEFT_BUMPER,
      RIGHT_BUMPER,
      START_BUTTON,
      BACK_BUTTON,
      LEFT_STICK_BUTT,
      RIGHT_STICK_BUTT;
  private POVButton UP, RIGHT, DOWN, LEFT;
  private Map<Integer, Axis> axisMap =
      new HashMap<>(
          Map.of(
              AxisNums.LEFT_X.ordinal(), new Axis(),
              AxisNums.LEFT_Y.ordinal(), new Axis(),
              AxisNums.L_TRIGGER.ordinal(), new Axis(),
              AxisNums.R_TRIGGER.ordinal(), new Axis(),
              AxisNums.RIGHT_X.ordinal(), new Axis(),
              AxisNums.RIGHT_Y.ordinal(), new Axis()));

  private static enum AxisNums {
    LEFT_X,
    LEFT_Y,
    L_TRIGGER,
    R_TRIGGER,
    RIGHT_X,
    RIGHT_Y
  }

  /**
   * Defines a decoratable axis on a joystick This class can be decorated by classes which define
   * functions to be applied to the axis
   */
  public static class Axis {

    /** The axis port */
    protected int port;
    /** The joystick */
    protected Joystick joystick = new Joystick(0);

    /** The axis instance */
    protected Axis instance;

    /**
     * Creates a new decoratable axis
     *
     * @param port the axis port
     * @param joystick the joystick
     */
    public Axis(int port, Joystick joystick) {
      this.port = port;
      this.joystick = joystick;
    }

    /** Creates a new decoratable axis */
    public Axis() {}

    /**
     * Gets the axis value
     *
     * @return the axis value
     */
    public double get() {
      return joystick.getRawAxis(port);
    }

    /**
     * Decorates the axis with a negation
     *
     * @return the negated axis decorator
     */
    public Axis negate() {
      return new Negate(this);
    }

    /**
     * Decorates the axis with a negation
     *
     * @param invert whether to invert the axis
     * @return the negated axis decorator
     */
    public Axis negate(boolean invert) {
      return new Negate(this, invert);
    }

    /**
     * Decorates the axis with a cubing function
     *
     * @return the curve axis decorator
     */
    public Axis cubed() {
      return new CurvePower(this);
    }

    /**
     * Decorates the axis with a curve at the given exponent
     *
     * @param power the curve exponent power
     * @return the curve axis decorator
     */
    public Axis curvePower(double power) {
      return new CurvePower(this, power);
    }

    /**
     * Decorates the axis with a deadzone
     *
     * @param deadzone the deadzone
     * @return the deadzone axis decorator
     */
    public Axis deadzone(double deadzone) {
      return new Deadzone(this, deadzone);
    }

    /**
     * Decorates the axis with a hard limit
     *
     * @param limit the limit
     * @return the hard limit axis decorator
     */
    public Axis limit(double limit) {
      return new HardLimit(this, limit);
    }

    /**
     * Decorates the axis with a rate limiter
     *
     * @param limit the limit
     * @return the rate limited axis decorator
     */
    public Axis rate(double limit) {
      return new ChangeRate(this, limit);
    }

    /**
     * Decorates the axis with a scaling
     *
     * @param scale the scale
     * @return the scaled axis decorator
     */
    public Axis scale(double scale) {
      return new Scale(this, scale);
    }
  }

  private static class Negate extends Axis {
    boolean invert = true;

    public Negate(Axis axis) {
      instance = axis;
    }

    public Negate(Axis axis, boolean invert) {
      instance = axis;
      this.invert = invert;
    }

    public double get() {
      return invert ? -instance.get() : instance.get();
    }
  }

  private static class CurvePower extends Axis {
    double power = 3.0;

    public CurvePower(Axis axis) {
      instance = axis;
    }

    public CurvePower(Axis axis, double power) {
      instance = axis;
      this.power = power;
    }

    public double get() {
      return Math.pow(instance.get(), power);
    }
  }

  private static class Scale extends Axis {
    double scale;

    public Scale(Axis axis, double scale) {
      instance = axis;
      this.scale = scale;
    }

    public double get() {
      return scale * instance.get();
    }
  }

  private static class Deadzone extends Axis {
    double deadzone;

    public Deadzone(Axis axis, double deadzone) {
      instance = axis;
      this.deadzone = deadzone;
    }

    public double get() {
      double input = instance.get();
      if (input > -deadzone && input < deadzone) {
        return 0.0;
      }
      return input;
    }
  }

  /** Decorates an axis with a hard limit */
  public static class HardLimit extends Axis {
    double limit;

    /**
     * Decorates an axis with a hard limit
     *
     * @param axis the axis
     * @param limit the limit
     */
    public HardLimit(Axis axis, double limit) {
      instance = axis;
      this.limit = limit;
    }

    /**
     * Get the hard limited value
     *
     * @return the hard limited value
     */
    public double get() {
      double input = instance.get();
      if (input > limit) {
        return limit;
      }
      if (input < -limit) {
        return -limit;
      }

      return input;
    }
  }

  /** Decorates an axis with a rate limiter */
  public static class ChangeRate extends Axis {
    SlewRateLimiter rateLimiter;

    /**
     * Decorates an axis with a rate limiter
     *
     * @param axis the axis
     * @param limit the limit
     */
    public ChangeRate(Axis axis, double limit) {
      instance = axis;
      this.rateLimiter = new SlewRateLimiter(limit);
    }

    /**
     * Get the rate limited value
     *
     * @return the rate limited value
     */
    public double get() {
      double input = instance.get();
      return rateLimiter.calculate(input);
    }
  }

  private static enum ButtonNums {
    NO_BUTTON,
    A_BUTTON,
    B_BUTTON,
    X_BUTTON,
    Y_BUTTON,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    BACK_BUTTON,
    START_BUTTON,
    LEFT_STICK_BUTT,
    RIGHT_STICK_BUTT;
  }

  /** Default joystick ports */
  public static enum JoystickPorts {
    /** Port 0 */
    ZERO,
    /** Port 1 */
    ONE,
    /** Port 2 */
    TWO,
    /** Port 3 */
    THREE
  }

  private static enum POVDirs {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    public int direction;

    private POVDirs(int direction) {
      this.direction = direction;
    }
  }

  /**
   * Constructor for a controller
   *
   * @param port the port of the controller
   */
  public Controller(int port) {
    joystick = new Joystick(port);
  }

  /**
   * Constructor for a controller
   *
   * @param port the port of the controller
   * @param single true if you want the controller to be single controller mode
   */
  public Controller(int port, boolean single) {
    joystick = new Joystick(port);
    singleControllerMode = single;
  }

  /**
   * Sets the rumble of the controller
   *
   * @param power the rumble to set
   */
  public void setRumble(double power) {
    joystick.setRumble(RumbleType.kBothRumble, power);
  }

  /**
   * @return true if the controller is plugged in
   */
  public boolean isPluggedIn() {
    return HIDType.kUnknown != joystick.getType();
  }

  /**
   * Sets the single controller mode This mode is used to indicate that there is actually only one
   * controller
   *
   * @param single true if you want the controller to be single controller mode
   */
  public void setSingleControllerMode(boolean single) {
    singleControllerMode = single;
  }

  /**
   * Gets the single controller mode This mode is used to indicate that there is actually only one
   * controller
   *
   * @return true if the controller is in single controller mode
   */
  public boolean isSingleControllerMode() {
    return singleControllerMode;
  }

  /**
   * Sets the left Y axis to use a particular decorated axis object
   *
   * @param yAxis the decorated axis object
   */
  public void setLeftYAxis(Axis yAxis) {
    axisMap.put(AxisNums.LEFT_Y.ordinal(), yAxis);
  }

  /**
   * Sets the left X axis to use a particular decorated axis object
   *
   * @param xAxis the decorated axis object
   */
  public void setLeftXAxis(Axis xAxis) {
    axisMap.put(AxisNums.LEFT_X.ordinal(), xAxis);
  }

  /**
   * Sets the right Y axis to use a particular decorated axis object
   *
   * @param yAxis the decorated axis object
   */
  public void setRightYAxis(Axis yAxis) {
    axisMap.put(AxisNums.RIGHT_Y.ordinal(), yAxis);
  }

  /**
   * Sets the right X axis to use a particular decorated axis object
   *
   * @param xAxis the decorated axis object
   */
  public void setRightXAxis(Axis xAxis) {
    axisMap.put(AxisNums.RIGHT_X.ordinal(), xAxis);
  }

  /**
   * Sets the left trigger axis to use a particular decorated axis object
   *
   * @param leftTriggerAxis the decorated axis object
   */
  public void setLeftTrigger(Axis leftTriggerAxis) {
    axisMap.put(AxisNums.L_TRIGGER.ordinal(), leftTriggerAxis);
  }

  /**
   * Sets the right trigger axis to use a particular decorated axis object
   *
   * @param rightTriggerAxis the decorated axis object
   */
  public void setRightTrigger(Axis rightTriggerAxis) {
    axisMap.put(AxisNums.R_TRIGGER.ordinal(), rightTriggerAxis);
  }

  /**
   * Creates a new decoratable axis for the left Y axis
   *
   * @return the new axis
   */
  public Axis createLeftYAxis() {
    return new Axis(AxisNums.LEFT_Y.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis for the left X axis
   *
   * @return the new axis
   */
  public Axis createLeftXAxis() {
    return new Axis(AxisNums.LEFT_X.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis for the right Y axis
   *
   * @return the new axis
   */
  public Axis createRightYAxis() {
    return new Axis(AxisNums.RIGHT_Y.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis for the right X axis
   *
   * @return the new axis
   */
  public Axis createRightXAxis() {
    return new Axis(AxisNums.RIGHT_X.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis for the left trigger
   *
   * @return the new axis
   */
  public Axis createLeftTrigger() {
    return new Axis(AxisNums.L_TRIGGER.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis for the right trigger
   *
   * @return the new axis
   */
  public Axis createRightTrigger() {
    return new Axis(AxisNums.R_TRIGGER.ordinal(), joystick);
  }

  /**
   * Creates a new decoratable axis
   *
   * @param channel the channel of the axis
   * @return the new axis
   */
  public Axis createAxis(int channel) {
    Axis axis = new Axis(channel, joystick);
    axisMap.put(channel, axis);
    return axis;
  }

  /**
   * Sets an axis on the controller
   *
   * @param channel the channel of the decoratable axis
   * @param axis the axis
   */
  public void setAxis(int channel, Axis axis) {
    axisMap.put(channel, axis);
  }

  /**
   * Gets an axis on the controller
   *
   * @param channel the channel of the axis
   * @return the axis
   */
  public Axis getAxis(int channel) {
    return axisMap.get(channel);
  }

  /**
   * Gets the current value of an axis
   *
   * @param channel the channel of the axis
   * @return the current value
   */
  public double getAxisValue(int channel) {
    Axis axis = axisMap.get(channel);
    if (null == axis) {
      return 0.0;
    }
    return axis.get();
  }

  /**
   * Gets the current value of the left Y axis
   *
   * @return the current value
   */
  public double getLeftYAxis() {
    return axisMap.get(AxisNums.LEFT_Y.ordinal()).get();
  }

  /**
   * Gets the current value of the left X axis
   *
   * @return the current value
   */
  public double getLeftXAxis() {
    return axisMap.get(AxisNums.LEFT_X.ordinal()).get();
  }

  /**
   * Gets the current value of the right Y axis
   *
   * @return the current value
   */
  public double getRightYAxis() {
    return axisMap.get(AxisNums.RIGHT_Y.ordinal()).get();
  }

  /**
   * Gets the current value of the right X axis
   *
   * @return the current value
   */
  public double getRightXAxis() {
    return axisMap.get(AxisNums.RIGHT_X.ordinal()).get();
  }

  /**
   * Gets the current value of the left trigger
   *
   * @return the current value
   */
  public double getLeftTrigger() {
    return axisMap.get(AxisNums.L_TRIGGER.ordinal()).get();
  }

  /**
   * Gets the current value of the right trigger
   *
   * @return the current value
   */
  public double getRightTrigger() {
    return axisMap.get(AxisNums.R_TRIGGER.ordinal()).get();
  }

  /**
   * Creates a new custom button
   *
   * @param buttonNum the button number
   * @return the new button
   */
  public JoystickButton createCustomButton(int buttonNum) {
    return new JoystickButton(joystick, buttonNum);
  }

  /**
   * Creates a new A button
   *
   * @return the new button
   */
  public JoystickButton createAButton() {
    A_BUTTON = new JoystickButton(joystick, ButtonNums.A_BUTTON.ordinal());
    return A_BUTTON;
  }

  /**
   * Creates a new B button
   *
   * @return the new button
   */
  public JoystickButton createBButton() {
    B_BUTTON = new JoystickButton(joystick, ButtonNums.B_BUTTON.ordinal());
    return B_BUTTON;
  }

  /**
   * Creates a new X button
   *
   * @return the new button
   */
  public JoystickButton createXButton() {
    X_BUTTON = new JoystickButton(joystick, ButtonNums.X_BUTTON.ordinal());
    return X_BUTTON;
  }

  /**
   * Creates a new Y button
   *
   * @return the new button
   */
  public JoystickButton createYButton() {
    Y_BUTTON = new JoystickButton(joystick, ButtonNums.Y_BUTTON.ordinal());
    return Y_BUTTON;
  }

  /**
   * Creates a new left bumper
   *
   * @return the new button
   */
  public JoystickButton createLeftBumper() {
    LEFT_BUMPER = new JoystickButton(joystick, ButtonNums.LEFT_BUMPER.ordinal());
    return LEFT_BUMPER;
  }

  /**
   * Creates a new right bumper
   *
   * @return the new button
   */
  public JoystickButton createRightBumper() {
    RIGHT_BUMPER = new JoystickButton(joystick, ButtonNums.RIGHT_BUMPER.ordinal());
    return RIGHT_BUMPER;
  }

  /**
   * Creates a new start button
   *
   * @return the new button
   */
  public JoystickButton createStartButton() {
    START_BUTTON = new JoystickButton(joystick, ButtonNums.START_BUTTON.ordinal());
    return START_BUTTON;
  }

  /**
   * Creates a new back button
   *
   * @return the new button
   */
  public JoystickButton createBackButton() {
    BACK_BUTTON = new JoystickButton(joystick, ButtonNums.BACK_BUTTON.ordinal());
    return BACK_BUTTON;
  }

  /**
   * Creates a new left stick button
   *
   * @return the new button
   */
  public JoystickButton createLeftStickButton() {
    LEFT_STICK_BUTT = new JoystickButton(joystick, ButtonNums.LEFT_STICK_BUTT.ordinal());
    return LEFT_STICK_BUTT;
  }

  /**
   * Creates a new right stick button
   *
   * @return the new button
   */
  public JoystickButton createRightStickButton() {
    RIGHT_STICK_BUTT = new JoystickButton(joystick, ButtonNums.RIGHT_STICK_BUTT.ordinal());
    return RIGHT_STICK_BUTT;
  }

  /**
   * Creates a new up Pov button
   *
   * @return the new button
   */
  public POVButton createUpPovButton() {
    UP = new POVButton(joystick, POVDirs.UP.direction);
    return UP;
  }

  /**
   * Creates a new down Pov button
   *
   * @return the new button
   */
  public POVButton createDownPovButton() {
    DOWN = new POVButton(joystick, POVDirs.DOWN.direction);
    return DOWN;
  }

  /**
   * Creates a new left Pov button
   *
   * @return the new button
   */
  public POVButton createLeftPovButton() {
    LEFT = new POVButton(joystick, POVDirs.LEFT.direction);
    return LEFT;
  }

  /**
   * Creates a new right Pov button
   *
   * @return the new button
   */
  public POVButton createRightPovButton() {
    RIGHT = new POVButton(joystick, POVDirs.RIGHT.direction);
    return RIGHT;
  }
}
