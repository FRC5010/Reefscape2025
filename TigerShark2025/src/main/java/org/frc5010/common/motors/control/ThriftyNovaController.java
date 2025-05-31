// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.EncoderType;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class ThriftyNovaController extends GenericPIDController {
    /** The controller */
    protected ThriftyNova controller;
    /** The reference */
    private double reference = 0.0;
    /** The tolerance */
    private double tolerance = 0.0;
    /** The PIDF configuration */
    private GenericPID pidfConfig = new GenericPID(0, 0, 0);
    PIDControlType controlType = PIDControlType.DUTY_CYCLE;

    public ThriftyNovaController(ThriftyNova controller) {
        this.controller = controller;
    }

    @Override
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    @Override
    public double getTolerance() {
        return tolerance;
    }

    @Override
    public void setValues(GenericPID pid) {
        PIDController pidc = new PIDController(pid.getkP(), pid.getkI(), pid.getkD());
        pidfConfig = new GenericPID(pid.getkP(), pid.getkI(), pid.getkD());
   //     controller.pid0.setPID(pidc);
    }

    @Override
    public void setP(double p) {
        pidfConfig.setkP(p);
        controller.pid0.setP(p);
    }

    @Override
    public void setI(double i) {
        pidfConfig.setkI(i);
        controller.pid0.setI(i);
    }

    @Override
    public void setD(double d) {
        pidfConfig.setkD(d);
        controller.pid0.setD(d);
    }

    @Override
    public void setF(double f) {
        pidfConfig.setkF(f);
        controller.pid0.setFF(f);
    }

    @Override
    public void setIZone(double iZone) {
        pidfConfig.setIZone(iZone);
        controller.pid0.setIZone(iZone);
    }

    @Override
    public void setOutputRange(double min, double max) {
    }

    @Override
    public void setReference(double reference) {
        this.reference = reference;
        switch (controlType) {
            case DUTY_CYCLE: {
                controller.set(reference);
                break;
            }
            case VOLTAGE: {
                controller.setVoltage(reference);
                break;
            }
            case VELOCITY: {
                controller.setVelocity(reference);
                break;
            }
            case POSITION: {
                controller.setPosition(reference);
                break;
            }
            default: throw new UnsupportedOperationException("Control type " + controlType + " not supported");              
        }
    }
    

    @Override
    public void setReference(double reference, PIDControlType controlType, double feedforward) {
        this.controlType = controlType;
        controller.pid0.setFF(feedforward);
        setReference(reference);
    }

    @Override
    public void setControlType(PIDControlType controlType) {
        this.controlType = controlType;
    }

    @Override
    public GenericPID getValues() {
        return pidfConfig;
    }

    @Override
    public double getP() {
        return pidfConfig.getkP();
    }

    @Override
    public double getI() {
        return pidfConfig.getkI();
    }

    @Override
    public double getD() {
        return pidfConfig.getkD();
    }

    @Override
    public double getF() {
        return pidfConfig.getkF();
    }

    @Override
    public double getIZone() {
        return pidfConfig.getIZone();
    }

    @Override
    public double getReference() {
        return reference;
    }

    @Override
    public PIDControlType getControlType() {
        return controlType;
    }

    @Override
    public boolean isAtTarget() {
        switch (controlType) {
            case DUTY_CYCLE: {
                return controller.get() == reference;
            }
            case VOLTAGE: {
                return controller.getVoltage() == reference;
            }
            case VELOCITY: {
                return controller.getVelocity() == reference;
            }
            case POSITION: {
                return controller.getPosition() == reference;
            }
            default: throw new UnsupportedOperationException("Control type " + controlType + " not supported");              
        }
    }   

    @Override
    public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
        controller.useEncoderType(EncoderType.ABS);
        controller.setEncoderPosition(offset);
    }

    @Override
    public void setProfiledMaxVelocity(double maxVelocity) {
        throw new UnsupportedOperationException("Unimplemented method 'setProfiledMaxVelocity'");
    }

    @Override
    public void setProfiledMaxAcceleration(double maxAcceleration) {
        throw new UnsupportedOperationException("Unimplemented method 'setProfiledMaxAcceleration'");
    }

    @Override
    public void setMotorFeedFwd(MotorFeedFwdConstants motorConstants) {
    }

}
