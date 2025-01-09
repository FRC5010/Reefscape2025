// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class ButtonBoard extends Controller {

  // private Joystick joystick;
  // private boolean singleControllerMode;
  private List<JoystickButton> BUTTONS = new ArrayList<>();
  // private POVButton UP, RIGHT, DOWN, LEFT;
  // private Axis AxisX, AxisY;

  /**
   * Creates a new ButtonBoard.
   *
   * @param port the port number
   */
  public ButtonBoard(int port) {
    super(port);
  }

  /**
   * Sets the Y Axis
   *
   * @param yAxis the Y Axis
   */
  public void setYAxis(Axis yAxis) {
    super.setLeftYAxis(yAxis);
  }

  /**
   * Sets the X Axis
   *
   * @param xAxis the X Axis
   */
  public void setXAxis(Axis xAxis) {
    super.setLeftXAxis(xAxis);
  }

  /**
   * Creates a Y Axis
   *
   * @return the Y Axis
   */
  public Axis createYAxis() {
    return super.createLeftYAxis();
  }

  /**
   * Creates an X Axis
   *
   * @return the X Axis
   */
  public Axis createXAxis() {
    return super.createLeftXAxis();
  }

  /**
   * Gets the value of the Y Axis
   *
   * @return the Y Axis value as a double
   */
  public double getYAxis() {
    return super.getLeftYAxis();
  }

  /**
   * Gets the value of the X Axis
   *
   * @return the X Axis value as a double
   */
  public double getXAxis() {
    return super.getLeftXAxis();
  }

  /**
   * Creates a set of buttons
   *
   * @param numberOfButtons the number of buttons
   */
  public void createButtons(int numberOfButtons) {
    for (int i = 1; i < numberOfButtons; i++) {
      BUTTONS.add(super.createCustomButton(i));
    }
  }

  /**
   * Gets a button based on the button number
   *
   * @param button the button
   * @return the button
   */
  public JoystickButton getButton(int button) {
    return BUTTONS.get(button - 1);
  }
}
