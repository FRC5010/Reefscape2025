// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.auto;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RelayPIDAutoTuner extends Command {
  private double relayAmplitude;
  private Consumer<Double> valueConsumer;
  private Supplier<Double> valueSupplier;
  private final Timer timer = new Timer();
  private final ArrayList<Double> outputList = new ArrayList<>();
  private final ArrayList<Double> timeList = new ArrayList<>();
  private final ArrayList<Double> errorList = new ArrayList<>();
  private final ArrayList<Double> switchTimeList = new ArrayList<>();
  private double lastOutput = 0;
  private double zeroValue;
  
  /** Creates a new RelayPIDAutoTuner. */
  public RelayPIDAutoTuner(Consumer<Double> valueConsumer, Supplier<Double> valueSupplier, double relayAmplitude, Subsystem... requirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.relayAmplitude = relayAmplitude;
    this.valueConsumer = valueConsumer;
    this.valueSupplier = valueSupplier;
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    zeroValue = valueSupplier.get();
    timer.reset();
    timer.start();
    lastOutput = relayAmplitude;
    timeList.clear();
    outputList.clear();
    errorList.clear();
  }

  private double relay(double error) {
    return error < 0 ? relayAmplitude : -relayAmplitude;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = timer.get();
    double error = valueSupplier.get().doubleValue() - zeroValue;
    
    double relayValue = relay(error);
    if (relayValue * lastOutput < 0) {
      switchTimeList.add(time);
    }
    lastOutput = relay(error);
    valueConsumer.accept(lastOutput);
    SmartDashboard.putNumber("Error", error);

    outputList.add(lastOutput);
    errorList.add(error);
    timeList.add(time);
    
    SmartDashboard.putNumber("Relay Output", lastOutput);
    
  }

  private double calculateOscillationPeriod() {
    double average = 0;
    for (int i = 0; i < switchTimeList.size() - 1; i++) {
      average += switchTimeList.get(i + 1) - switchTimeList.get(i);
    }
    average /= switchTimeList.size() - 1;
    return average * 2;
  }

  private int[] getPeaks() {
    int[] peaks = new int[errorList.size()];
    for (int i = 1; i < errorList.size() - 1; i++) {
      if (errorList.get(i) > errorList.get(i - 1) && errorList.get(i) > errorList.get(i + 1)) {
        peaks[i] = 1;
      }
    }
    return peaks;
  }

  private double calculateAverageResponseAmplitude() {
    int[] peaks = getPeaks();
    double sum = 0;
    int count = 0;
    for (int i = 0; i < peaks.length; i++) {
      if (peaks[i] == 1) {
        sum += errorList.get(i);
        count++;
      }
    }
    return sum / count;
  }

  private void tunePID() {
    double uPeriod = calculateOscillationPeriod();
    double amplitude = calculateAverageResponseAmplitude();

    double uGain = 4 * relayAmplitude / (Math.PI * amplitude);

    SmartDashboard.putNumber("Ultimate Gain", uGain);
    SmartDashboard.putNumber("Ultimate Period", uPeriod);

    double kP = 0.2 * uGain;
    double kI = 2 * kP / uPeriod;
    double kD = kP * uPeriod / 3;
    SmartDashboard.putNumber("kP System", kP);
    SmartDashboard.putNumber("kI System", kI);
    SmartDashboard.putNumber("kD System", kD);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tunePID(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
