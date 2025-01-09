package org.frc5010.common.sensors.absolute_encoder;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.wpilibj.Alert;

/**
 * SparkMax absolute encoder, attached through the data port. Credit: YAGSL for the original code
 */
public class RevAbsoluteEncoder extends GenericAbsoluteEncoder {
  /** The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkMax. */
  public AbsoluteEncoder encoder;
  /** An {@link Alert} for if there is a failure configuring the encoder. */
  private Alert failureConfiguring;
  /** An {@link Alert} for if there is a failure configuring the encoder offset. */
  private Alert offsetFailure;
  private final AbsoluteEncoderConfig config;
  private final SparkBase motor;

  /**
   * Create the {@link RevAbsoluteEncoder} object as a duty cycle from the {@link SparkMax}
   * motor.
   *
   * @param motor Motor to create the encoder from.
   * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
   */
  public RevAbsoluteEncoder(SparkMax motor, int conversionFactor) {
    config = new AbsoluteEncoderConfig();
    failureConfiguring =
        new Alert(
            "Encoders",
            "Failure configuring SparkMax Analog Encoder",
            Alert.AlertType.kWarning);
    offsetFailure =
        new Alert(
            "Encoders", "Failure to set Absolute Encoder Offset", Alert.AlertType.kWarning);
    if (motor instanceof SparkMax) {
      this.motor = motor;
      encoder = this.motor.getAbsoluteEncoder();
      setPositionConversion(conversionFactor);
      setVelocityConversion(conversionFactor);
    } else {
      throw new RuntimeException("Motor given to instantiate SparkMaxEncoder is not a SparkMax");
    }
  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   */
  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    failureConfiguring.set(true);
  }

  /** Reset the encoder to factory defaults. */
  @Override
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  @Override
  public void clearStickyFaults() {
    // Do nothing
  }

  /**
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted) {
    config.inverted(inverted);
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getEncoder() {
    return encoder;
  }

  /**
   * Sets the Absolute Encoder Offset inside of the SparkMax's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setEncoderOffset(double offset) {
    REVLibError error = null;
    for (int i = 0; i < maximumRetries; i++) {
      config.zeroOffset(offset);
      if (error == REVLibError.kOk) {
        return true;
      }
    }
    offsetFailure.setText("Failure to set Absolute Encoder Offset Error: " + error);
    offsetFailure.set(true);
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void reset() {
    config.zeroOffset(0);
  }

  @Override
  public void setPositionConversion(double conversion) {
    config.positionConversionFactor(conversion);
  }

  @Override
  public void setVelocityConversion(double conversion) {
    config.velocityConversionFactor(conversion);
  }

  @Override
  public void setPosition(double position) {
    throw new IllegalArgumentException();
  }

  @Override
  public void setRate(double rate) {}

  @Override
  public void setInverted(boolean inverted) {
    config.inverted(inverted);
  }
}
