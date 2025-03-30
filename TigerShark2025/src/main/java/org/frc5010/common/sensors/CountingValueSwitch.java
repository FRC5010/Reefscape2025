// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.math.filter.LinearFilter;
import java.util.function.Supplier;

/** Add your docs here. */
public class CountingValueSwitch extends ValueSwitch {
  private LinearFilter valueFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private int counts;
  private int triggerCounts;


  public CountingValueSwitch(double threshold, Supplier<Double> value, double triggerThreshold, int countsToTrigger) {
    super(threshold, value, triggerThreshold);
    triggerCounts = countsToTrigger;
  }

  @Override
  public Boolean get() {
    boolean switchState = (valueFilter.calculate(valueSupplier.get()) - thresholdSupplier.get())
    > triggerThreshold;
    counts += switchState ? 1 : 0;
    boolean countsAchieved = counts > triggerCounts;
    if (countsAchieved) {
        counts = 0;
    }
    return countsAchieved;
  }

  public void resetCount() {
    counts = 0;
  }

  public void updateValue() {
    valueFilter.calculate(valueSupplier.get());
  }
}
