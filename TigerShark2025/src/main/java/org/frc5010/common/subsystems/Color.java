// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */

// ENUM for all colors
public enum Color {
  /** White */
  WHITE(255, 255, 255),
  /** Off */
  OFF(0, 0, 0),
  /** Red */
  RED(255, 0, 0),
  /** Green */
  GREEN(0, 255, 0),
  /** Blue */
  BLUE(0, 0, 255),
  /** Orange */
  ORANGE(255, 125, 00), // A500
  /** Purple */
  PURPLE(100, 0, 255),
  /** 5010 Orange */
  FIFTY_TEN_ORANGE(255, 80, 16),
  /** Yellow */
  YELLOW(210, 255, 0);

  private Color8Bit color;

  private Color(int red, int green, int blue) {

    // using the integers in the enum we make a new Color8Bit
    this.color = new Color8Bit(red, green, blue);
  }

  /**
   * Gets the color as a Color8Bit
   *
   * @return Color8Bit
   */
  public Color8Bit getColor8Bit() {
    // using the integers in each enum return the Color8bit
    return color;
  }

  /**
   * Converts from HSV to Color8Bit
   *
   * @param hue the hue
   * @param sat the saturation
   * @param value the value
   * @return Color8Bit
   */
  public static Color8Bit fromHSV(int hue, int sat, int value) {
    return new Color8Bit(edu.wpi.first.wpilibj.util.Color.fromHSV(hue, sat, value).toHexString());
  }

  /**
   * Gets a Color8Bit with percentage of intensity
   *
   * @param color the color
   * @param alphaPercent the percentage of intensity
   * @return Color8Bit
   */
  public static Color8Bit getColor8BitAlpha(Color8Bit color, double alphaPercent) {

    // change the intensity of the led color.
    // keeps it between 0 and 100 percent intensity
    alphaPercent = MathUtil.clamp(alphaPercent, 0, 85) / 100;
    int red = (int) (color.red * alphaPercent);
    int green = (int) (color.green * alphaPercent);
    int blue = (int) (color.blue * alphaPercent);

    return new Color8Bit(red, green, blue);
  }
}
