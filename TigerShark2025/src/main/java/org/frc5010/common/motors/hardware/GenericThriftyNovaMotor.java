// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.control.ThriftyNovaController;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.sensors.encoder.ThriftyNovaEncoder;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Add your docs here. */
public class GenericThriftyNovaMotor implements MotorController5010 {
    /** The motor controller */
    protected ThriftyNova motor;
    /** The maximum RPM */
    protected AngularVelocity maxRPM;
    /** The motor simulation type */
    protected DCMotor motorSim;

    /** The current limit */
    protected Current currentLimit;
    /** The internal encoder representation */
    protected GenericEncoder encoder;
    /** The internal simulation representation */
    protected SimulatedEncoder simEncoder;
    /** The internal PID controller representation */
    protected PIDController5010 controller;
    /** The configuration */
    protected Motor config;

    private GenericThriftyNovaMotor(int canId, Motor config, Current currentLimit) {
        this(canId, config);
        setCurrentLimit(currentLimit);
    }

    public GenericThriftyNovaMotor(int canId, Motor config) {
        motor = new ThriftyNova(canId);
        this.config = config;
        factoryDefaults();
        clearStickyFaults();
        setCurrentLimit(config.currentLimit);
        setMotorSimulationType(config.getMotorSimulationType());
        setMaxRPM(config.maxRpm);
        encoder = new ThriftyNovaEncoder(motor);
        simEncoder = new SimulatedEncoder(
                MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
        controller = new ThriftyNovaController(motor);
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
        simEncoder.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.disable();
    }

    @Override
    public void stopMotor() {
        motor.set(0);
    }

    @Override
    public MotorController5010 duplicate(int port) {
        return new GenericThriftyNovaMotor(port, config, currentLimit);
    }

    @Override
    public MotorController5010 setSlewRate(double rate) {
        motor.setRampUp(rate);
        motor.setRampDown(rate);
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        this.motor.follow(((ThriftyNova) motor.getMotor()).getID());
        return this;
    }

    @Override
    public MotorController5010 setFollow(MotorController5010 motor, boolean inverted) {
        this.motor.follow(((ThriftyNova) motor.getMotor()).getID());
        this.motor.setInverted(inverted);
        return this;
    }

    @Override
    public MotorController5010 invert(boolean inverted) {
        motor.setInverted(inverted);
        simEncoder.setInverted(inverted);
        return this;
    }

    @Override
    public MotorController5010 setCurrentLimit(Current limit) {
        this.currentLimit = limit;
        motor.setMaxCurrent(CurrentType.STATOR, limit.in(Amps));
        return this;
    }

    @Override
    public GenericEncoder getMotorEncoder() {
        return encoder;
    }

    @Override
    public GenericEncoder createMotorEncoder(int countsPerRev) {
        encoder.setPositionConversion(countsPerRev);
        simEncoder.setPositionConversion(countsPerRev);
        encoder.setVelocityConversion(countsPerRev / 60.0);
        simEncoder.setVelocityConversion(countsPerRev / 60.0);
        return encoder;
    }

    @Override
    public PIDController5010 getPIDController5010() {
        return controller;
    }

    @Override
    public Object getMotor() {
        return motor;
    }

    @Override
    public void factoryDefaults() {
    }

    @Override
    public SysIdRoutine getDefaultSysId(SubsystemBase subsystemBase) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDefaultSysId'");
    }

    @Override
    public DCMotor getMotorSimulationType() {
        return motorSim;
    }

    @Override
    public AngularVelocity getMaxRPM() {
        return maxRPM;
    }

    @Override
    public MotorController5010 setVoltageCompensation(double nominalVoltage) {
        motor.setVoltageCompensation(nominalVoltage);
        return this;
    }

    @Override
    public void clearStickyFaults() {
        motor.clearErrors();
    }

    @Override
    public MotorController5010 setMotorBrake(boolean isBrakeMode) {
        motor.setBrakeMode(isBrakeMode);
        return this;
    }

    @Override
    public void burnFlash() {
    }

    @Override
    public double getVoltage() {
        return motor.getVoltage();
    }

    @Override
    public double getAppliedOutput() {
        return getOutputCurrent() * getVoltage();
    }

    @Override
    public double getOutputCurrent() {
        return motor.getStatorCurrent();
    }

    @Override
    public void setMotorSimulationType(DCMotor motorSimulationType) {
        motorSim = motorSimulationType;
    }

    @Override
    public void simulationUpdate(Optional<Double> position, Double velocity) {
        simEncoder.setPosition(position.map(it -> it).orElse(0.0));
        simEncoder.setRate(velocity);
    }

    @Override
    public void setMaxRPM(AngularVelocity rpm) {
        maxRPM = rpm;
    }
}
