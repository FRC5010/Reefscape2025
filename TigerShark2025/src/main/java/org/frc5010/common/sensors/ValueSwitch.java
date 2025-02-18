package org.frc5010.common.sensors;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ValueSwitch {
  Supplier<Double> thresholdSupplier;
  Supplier<Double> valueSupplier;
  double triggerThreshold;

  public ValueSwitch() {
    this.thresholdSupplier = () -> 0.0;
    this.valueSupplier = () -> 0.0;
    this.triggerThreshold = 0.0;
  }

  public ValueSwitch(Supplier<Double> threshold, Supplier<Double> value, double triggerThreshold) {
    this.thresholdSupplier = threshold;
    this.valueSupplier = value;
    this.triggerThreshold = triggerThreshold;
  }

  public ValueSwitch(double threshold, Supplier<Double> value, double triggerThreshold) {
    this.thresholdSupplier = () -> threshold;
    this.valueSupplier = value;
    this.triggerThreshold = triggerThreshold;
  }

  public Trigger getTrigger() {
    return new Trigger(this::get);
  }

  public Boolean get() {
    return (valueSupplier.get() - thresholdSupplier.get()) > triggerThreshold;
  }

  public void setValueSupplier(Supplier<Double> value) {
    valueSupplier = value;
  }

  public void setThreshold(Supplier<Double> threshold) {
    thresholdSupplier = threshold;
  }

  public void setThreshold(double threshold) {
    thresholdSupplier = () -> threshold;
  }

  public void setTriggerThreshold(double threshold) {
    triggerThreshold = threshold;
  }
}
