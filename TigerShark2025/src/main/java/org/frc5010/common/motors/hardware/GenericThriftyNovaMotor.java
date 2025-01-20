// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.hardware;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Add your docs here. */
public class GenericThriftyNovaMotor implements MotorController5010 {
    /** The motor controller */
    protected ThriftyNova motor;
    protected AngularVelocity maxRPM;
    protected DCMotor motorSim;

    /** The current limit */
    protected Current currentLimit;

    public GenericThriftyNovaMotor(int canId) {
        motor = new ThriftyNova(canId);
    }

    public GenericThriftyNovaMotor(int canId, Current currentLimit) {
        motor = new ThriftyNova(canId);
        this.currentLimit = currentLimit;
        setCurrentLimit(currentLimit);
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
        return new GenericThriftyNovaMotor(port, currentLimit);
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
        return this;}

    @Override
    public MotorController5010 setCurrentLimit(Current limit) {
        this.currentLimit = limit;
        motor.setMaxCurrent(CurrentType.STATOR, limit.in(Amps));
        return this;
    }

    @Override
    public GenericEncoder getMotorEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorEncoder'");
    }

    @Override
    public GenericEncoder getMotorEncoder(int countsPerRev) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorEncoder'");
    }

    @Override
    public PIDController5010 getPIDController5010() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPIDController5010'");
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorSimulationType'");
    }

    @Override
    public AngularVelocity getMaxRPM() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMaxRPM'");
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
    public void simulationUpdate(Optional<Angle> position, AngularVelocity velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'simulationUpdate'");
    }

    @Override
    public void setMaxRPM(AngularVelocity rpm) {
        maxRPM = rpm;
    }

}
