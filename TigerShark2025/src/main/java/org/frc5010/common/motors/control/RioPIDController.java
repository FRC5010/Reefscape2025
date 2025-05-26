// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.control;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class RioPIDController extends GenericPIDController {
    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(0, 0));
    private double feedForward = 0;
    private PIDControlType controlType = PIDControlType.POSITION;

    private void executeSwitchedTask(Runnable task, Runnable profiledTask) {
        switch (controlType) {
            case PROFILED_POSITION:
            case PROFILED_VELOCITY:
                profiledTask.run();
                break;
            default:
                task.run();
                break;
        }
    }

    private double executeSwitchedGetter(DoubleSupplier task, DoubleSupplier profiledTask) {
        switch (controlType) {
            case PROFILED_POSITION:
            case PROFILED_VELOCITY:
                return profiledTask.getAsDouble();
            default:
                return task.getAsDouble();
        }
    }

    private boolean executeSwitchedGetter(BooleanSupplier task, BooleanSupplier profiledTask) {
        switch (controlType) {
            case PROFILED_POSITION:
            case PROFILED_VELOCITY:
                return profiledTask.getAsBoolean();
            default:
                return task.getAsBoolean();
        }
    }

    @Override
    public void setTolerance(double tolerance) {
        executeSwitchedTask(() -> pidController.setTolerance(tolerance),
                () -> profiledPIDController.setTolerance(tolerance));
    }

    @Override
    public double getTolerance() {
        return executeSwitchedGetter(() -> pidController.getErrorTolerance(),
                () -> profiledPIDController.getPositionTolerance());
    }

    @Override
    public void setValues(GenericPID pid) {
        executeSwitchedTask(() -> pidController.setPID(pid.getkP(), pid.getkI(), pid.getkD()),
                () -> profiledPIDController.setPID(pid.getkP(), pid.getkI(), pid.getkD()));
    }

    @Override
    public void setMotorFeedFwd(MotorFeedFwdConstants motorConstants) {
    }

    @Override
    public void setP(double p) {
        executeSwitchedTask(() -> pidController.setP(p), () -> profiledPIDController.setP(p));
    }

    @Override
    public void setI(double i) {
        executeSwitchedTask(() -> pidController.setI(i), () -> profiledPIDController.setI(i));
    }

    @Override
    public void setD(double d) {
        executeSwitchedTask(() -> pidController.setD(d), () -> profiledPIDController.setD(d));
    }

    @Override
    public void setF(double f) {
        feedForward = f;
    }

    @Override
    public void setIZone(double iZone) {
        executeSwitchedTask(() -> pidController.setIZone(iZone), () -> profiledPIDController.setIZone(iZone));
    }

    @Override
    public void setOutputRange(double min, double max) {
    }

    @Override
    public void setReference(double reference) {
        executeSwitchedTask(() -> pidController.setSetpoint(reference), () -> profiledPIDController.setGoal(reference));
    }

    @Override
    public void setReference(double reference, PIDControlType controlType, double feedforward) {
        setControlType(controlType);
        this.feedForward = feedforward;
        executeSwitchedTask(() -> pidController.setSetpoint(reference), () -> profiledPIDController.setGoal(reference));
    }

    @Override
    public void resetController(double position, double velocity) {
        executeSwitchedTask(() -> {
        },
                () -> profiledPIDController.reset(position, velocity));
    }

    @Override
    public void setControlType(PIDControlType controlType) {
        this.controlType = controlType;
    }

    @Override
    public void setProfiledMaxVelocity(double maxVelocity) {
        profiledPIDController.setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity, profiledPIDController.getConstraints().maxAcceleration));
    }

    @Override
    public void setProfiledMaxAcceleration(double maxAcceleration) {
        profiledPIDController.setConstraints(
                new TrapezoidProfile.Constraints(profiledPIDController.getConstraints().maxVelocity, maxAcceleration));
    }

    @Override
    public GenericPID getValues() {
        GenericPID values = new GenericPID(
                executeSwitchedGetter(() -> pidController.getP(), () -> profiledPIDController.getP()),
                executeSwitchedGetter(() -> pidController.getI(), () -> profiledPIDController.getI()),
                executeSwitchedGetter(() -> pidController.getD(), () -> profiledPIDController.getD()));
        return values;
    }

    @Override
    public double getP() {
        return executeSwitchedGetter(() -> pidController.getP(), () -> profiledPIDController.getP());
    }

    @Override
    public double getI() {
        return executeSwitchedGetter(() -> pidController.getI(), () -> profiledPIDController.getI());
    }

    @Override
    public double getD() {
        return executeSwitchedGetter(() -> pidController.getD(), () -> profiledPIDController.getD());
    }

    @Override
    public double getF() {
        return feedForward;
    }

    @Override
    public double getIZone() {
        return executeSwitchedGetter(() -> pidController.getIZone(), () -> profiledPIDController.getIZone());
    }

    @Override
    public double getReference() {
        return executeSwitchedGetter(() -> pidController.getSetpoint(), () -> profiledPIDController.getGoal().position);
    }

    @Override
    public double getReferenceVelocity() {
        return executeSwitchedGetter(() -> 0, () -> profiledPIDController.getGoal().velocity);
    }

    @Override
    public PIDControlType getControlType() {
        return controlType;
    }

    @Override
    public boolean isAtTarget() {
        return executeSwitchedGetter(() -> pidController.atSetpoint(), () -> profiledPIDController.atSetpoint());
    }

    @Override
    public double calculateControlEffort(double current) {
        return executeSwitchedGetter(() -> {
            double control = pidController.calculate(current);
            return control;
        }, () -> {
            double control = profiledPIDController.calculate(current);
            return control;
        });
    }

    @Override
    public void configureAbsoluteControl(double offset, boolean inverted, double min, double max) {
        throw new UnsupportedOperationException("Unimplemented method 'configureAbsoluteControl'");
    }

}