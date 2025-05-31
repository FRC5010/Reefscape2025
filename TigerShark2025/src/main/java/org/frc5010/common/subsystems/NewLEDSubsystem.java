// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewLEDSubsystem extends SubsystemBase {
  private int kPort = 0, kLength = 0;
  private Distance kLEDSpacing = Meters.of(0.0);
  private LEDPattern pattern = LEDPattern.solid(Color.kOrange);

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  public NewLEDSubsystem(int kPort, int kLength, Distance physicalLength) {
    this.kPort = kPort;
    this.kLength = kLength;
    this.kLEDSpacing = physicalLength.div(kLength);

    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern(() -> getCurrentPattern()).ignoringDisable(true));
  }

  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern;
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(Supplier<LEDPattern> pattern) {
    return run(() -> pattern.get().applyTo(m_buffer));
  }

  public LEDPattern getRainbowPattern(double percentScrollingSpeed) {
    return LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
  }

  public LEDPattern getSolidPattern(Color color) {
    return LEDPattern.solid(color);
  }

  public LEDPattern getMaskedPattern(LEDPattern basePattern, double percentVisible, double percentScrollingSpeed) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, percentVisible, Color.kBlack);
    LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
    return basePattern.mask(mask);
  }

  public LEDPattern getBand(LEDPattern basePattern, double bandCenter, double percentWidth, double percentScrollingSpeed) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kBlack, bandCenter - (percentWidth / 2), Color.kWhite, bandCenter - (percentWidth / 2), Color.kBlack);
    LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(percentScrollingSpeed));
    return basePattern.mask(mask);
  }

  public LEDPattern getBlinkingPattern(LEDPattern basePattern, Time blinkInterval) {
    return basePattern.blink(blinkInterval);
  }

  public LEDPattern getCurrentPattern() {
    return pattern;
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }
}