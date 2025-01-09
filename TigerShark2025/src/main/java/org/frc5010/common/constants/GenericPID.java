// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Class for holding PID constants */
public class GenericPID {
  /** The proportional constant */
  private double kP;

  /** The integral constant */
  private double kI;

  /** The derivative constant */
  private double kD;

  /** The feedforward constant */
  private double kF;

  /** The integral zone */
  private double iZone;

  /**
   * Constructor for GenericPID
   *
   * @param kP - proportional
   * @param kI - integral
   * @param kD - derivative
   */
  public GenericPID(double kP, double kI, double kD) {

    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /**
   * Returns the value of the proportional constant.
   *
   * @return the value of the proportional constant
   */
  public double getkP() {
    return kP;
  }

  /**
   * Sets the value of the proportional constant.
   *
   * @param kP the new value of the proportional constant
   */
  public void setkP(double kP) {
    this.kP = kP;
  }

  /**
   * Returns the value of the integral constant.
   *
   * @return the value of the integral constant
   */
  public double getkI() {
    return kI;
  }

  /**
   * Sets the value of the integral constant.
   *
   * @param kI the new value of the integral constant
   */
  public void setkI(double kI) {
    this.kI = kI;
  }

  /**
   * Returns the value of the derivative constant.
   *
   * @return the value of the derivative constant
   */
  public double getkD() {
    return kD;
  }

  /**
   * Sets the value of the derivative constant.
   *
   * @param kD the new value of the derivative constant
   */
  public void setkD(double kD) {
    this.kD = kD;
  }

  /**
   * Returns the value of the feedforward constant.
   *
   * @return the value of the feedforward constant
   */
  public double getkF() {
    return kF;
  }

  /**
   * Sets the value of the feedforward constant.
   *
   * @param kF the new value of the feedforward constant
   */
  public void setkF(double kF) {
    this.kF = kF;
  }

  /**
   * Returns the value of the integral zone.
   *
   * @return the value of the integral zone
   */
  public double getIZone() {
    return iZone;
  }

  /**
   * Sets the value of the integral zone.
   *
   * @param iZone the new value of the integral zone
   */
  public void setIZone(double iZone) {
    this.iZone = iZone;
  }
}
