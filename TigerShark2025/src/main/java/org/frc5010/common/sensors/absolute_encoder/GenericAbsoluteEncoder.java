package org.frc5010.common.sensors.absolute_encoder;

import org.frc5010.common.sensors.encoder.GenericEncoder;

/**
 * Swerve abstraction class to define a standard interface with absolute encoders for swerve
 * modules. Credit: YAGSL for the original code
 */
public abstract class GenericAbsoluteEncoder implements GenericEncoder {

  /**
   * The maximum amount of times the swerve encoder will attempt to configure itself if failures
   * occur.
   */
  public final int maximumRetries = 5;
  /** Last angle reading was faulty. */
  public boolean readingError = false;

  /** Reset the encoder to factory defaults. */
  public abstract void factoryDefault();

  /** Clear sticky faults on the encoder. */
  public abstract void clearStickyFaults();

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  public abstract void configure(boolean inverted);

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public abstract double getPosition();

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  public abstract Object getEncoder();

  /**
   * Sets the Absolute Encoder offset at the Encoder Level.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point in degrees.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  public abstract boolean setEncoderOffset(double offset);

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public abstract double getVelocity();
}
