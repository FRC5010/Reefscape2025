// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.print.attribute.PrintServiceAttributeSet;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.function.VerticalPositionControlMotor;
import org.frc5010.common.telemetry.DisplayLength;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;

/** Add your docs here. */
public class ElevatorSystem extends GenericSubsystem {
    protected VerticalPositionControlMotor elevator;
    protected FollowerMotor elevatorFollower;
    protected PIDControlType controlType = PIDControlType.NONE;
    protected Distance safeDistance = Inches.of(6);
    public DisplayLength BOTTOM = displayValues.makeConfigLength(Position.BOTTOM.name());
    public DisplayLength LOAD = displayValues.makeConfigLength(Position.LOAD.name());
    public DisplayLength PROCESSOR = displayValues.makeConfigLength(Position.PROCESSOR.name());
    public DisplayLength L1 = displayValues.makeConfigLength(Position.L1.name());
    public DisplayLength L2Algae = displayValues.makeConfigLength(Position.L2Algae.name());
    public DisplayLength L2Shoot = displayValues.makeConfigLength(Position.L2Shoot.name());
    public DisplayLength L2 = displayValues.makeConfigLength(Position.L2.name());
    public DisplayLength L3Algae = displayValues.makeConfigLength(Position.L3Algae.name());
    public DisplayLength L3Shoot = displayValues.makeConfigLength(Position.L3Shoot.name());
    public DisplayLength L3 = displayValues.makeConfigLength(Position.L3.name());
    public DisplayLength L4Shoot = displayValues.makeConfigLength(Position.L4Shoot.name());
    public DisplayLength L4 = displayValues.makeConfigLength(Position.L4.name());
    public DisplayLength NET = displayValues.makeConfigLength(Position.NET.name());
    
    public static enum Position {
        BOTTOM(Meters.of(0.0)),
        LOAD(Meters.of(0.09)),
        PROCESSOR(Meters.of(0.15)),
        L1(Meters.of(0.72)),
        L2Algae(Meters.of(0.11)),
        L2Shoot(Meters.of(0.87)),
        L2(Meters.of(1.0)),
        L3Algae(Meters.of(1.1)),
        L3Shoot(Meters.of(1.27)),
        L3(Meters.of(1.297)),
        L4Shoot(Meters.of(1.83)),
        L4(Meters.of(1.84)),
        NET(Meters.of(1.9));

        private final Distance position;

        private Position(Distance position) {
            this.position = position;
        }

        public Distance position() {
            return position;
        }
    }

    private double lastError;
    private double lastTimestamp = 0;

    public ElevatorSystem(Mechanism2d mechanismSimulation) {
        if (0 == BOTTOM.getLength().in(Meters)) BOTTOM.setLength(Position.BOTTOM.position());
        if (0 == LOAD.getLength().in(Meters)) LOAD.setLength(Position.LOAD.position());
        if (0 == PROCESSOR.getLength().in(Meters)) PROCESSOR.setLength(Position.PROCESSOR.position());
        if (0 == L1.getLength().in(Meters)) L1.setLength(Position.L1.position());
        if (0 == L2Algae.getLength().in(Meters)) L2Algae.setLength(Position.L2Algae.position());
        if (0 == L2Shoot.getLength().in(Meters)) L2Shoot.setLength(Position.L2Shoot.position());
        if (0 == L2.getLength().in(Meters)) L2.setLength(Position.L2.position());
        if (0 == L3Algae.getLength().in(Meters)) L3Algae.setLength(Position.L3Algae.position());
        if (0 == L3Shoot.getLength().in(Meters)) L3Shoot.setLength(Position.L3Shoot.position());
        if (0 == L3.getLength().in(Meters)) L3.setLength(Position.L3.position());
        if (0 == L4Shoot.getLength().in(Meters)) L4Shoot.setLength(Position.L4Shoot.position());
        if (0 == L4.getLength().in(Meters)) L4.setLength(Position.L4.position());
        if (0 == NET.getLength().in(Meters)) NET.setLength(Position.NET.position());

        elevator = new VerticalPositionControlMotor(MotorFactory.TalonFX(9, Motor.KrakenX60), "elevator",
                displayValues);
        elevatorFollower = new FollowerMotor(MotorFactory.TalonFX(10, Motor.KrakenX60),
        elevator, "elevatorFollower", true);
        elevator.setControlType(controlType);

        elevator.setMotorBrake(true);
        elevatorFollower.setMotorBrake(true);
        elevatorFollower.setCurrentLimit(Amps.of(100));
        
        elevator.setupSimulatedMotor(6, Pounds.of(15), Inches.of(1.1), Meters.of(0), Inches.of(83.475 - 6.725),
                Meters.of(0),
                Meters.of(0.2), 0.263672);
        elevator.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(5.75).in(Meters), Inches.of(4.75).in(Meters), Inches.of(6.725).in(Meters)),
                new Rotation3d()));
        elevator.setCurrentLimit(Amps.of(100));
        elevator.setMotorFeedFwd(new MotorFeedFwdConstants(0.26329, 0.38506, 0.04261));
        elevator.setProfiledMaxVelocity(2.0);
        elevator.setProfiledMaxAcceleration(5);
        elevator.setValues(new GenericPID(20.0, 0.0, 2.0));
        elevator.setOutputRange(-1, 1);

        elevator.getMotorEncoder().setPosition(LOAD.getLengthInMeters());

        // Tell the elevator to run the motor in reverse because the simulator thinks CW
        // is upwards
        if (RobotBase.isSimulation()) {
            elevator.invert(true);
            // Tell the simulator that the motor is CW.
            elevator.getMotorEncoder().setInverted(true);


        }
        
        elevator.burnFlash();
    }

    public Boolean validSpeed(double speed) {
        if ((speed > 0 && !elevator.isAtMax()) || (speed < 0 && !elevator.isAtMin())) {
            return true;
        }
        return false;
    }
    

    public Command elevatorSysIdCommand() {
        return elevator.getSysIdCommand(this);    
    }

    public double safeSpeed(double speed) {
        if (!validSpeed(speed)) {
            return 0.0;
        }
        if ((speed > 0 && elevator.isCloseToMax(safeDistance)) || (speed < 0 && elevator.isCloseToMin(safeDistance))) {
            return Math.max(speed*0.1, Math.signum(speed)*0.15);
        }
        return speed;
    }

    public void elevatorSpeed(double speed) {

        if (elevator.getControlType() != PIDControlType.NONE && speed != 0) {
            elevator.setControlType(PIDControlType.NONE);
        }
        if (PIDControlType.NONE == elevator.getControlType()) {
            elevator.set(safeSpeed(speed));
        }
    }

    public Command profiledBangBangCmd(Distance position) {
        return Commands.run(() -> {

            elevator.setReference(position.in(Meters));
            double difference = position.in(Meters) - elevator.getPosition();
            double sign = Math.signum(difference);
            double effort = 0.8;
            if (Math.abs(difference) < safeDistance.in(Meters)) {
                effort *= Math.abs(difference) / safeDistance.in(Meters);
                effort = Math.max(effort, 0.001);
                if (Math.abs(difference) < 0.002) {
                    effort = 0;
                }
            }
            effort *= sign;
            elevator.set(effort);
        }, this);
    }

    public Command pidControlCommand(Distance position) {
        return Commands.run(() -> {
            double currentError = position.in(Meters) - elevator.getPosition();
            double currentTimestamp = RobotController.getFPGATime() / 10E3;
            double d = (currentError - lastError) / (currentTimestamp - lastTimestamp);
            double effort = MathUtil.clamp(elevator.getP() * currentError + elevator.getD() * d, -0.5, 0.5);
            elevator.set(effort);
            lastTimestamp = currentTimestamp;
            lastError = currentError;
        }, this);
    }

    public void setElevatorPosition(Distance position) {
        if (elevator.getControlType() != controlType) {
            elevator.setControlType(controlType);
        }
        if (controlType == elevator.getControlType()) {
            elevator.setReference(position.in(Meters));
        }
    }


    public Command zeroElevator() {
        return Commands.runOnce(() -> elevator.getMotorEncoder().setPosition(0.09));
    }

    public Command basicSuppliersMovement(DoubleSupplier speed) {
        return Commands.run(
            () -> elevatorSpeed(speed.getAsDouble()), this);

    }

    public Trigger isAtTarget() {
        return new Trigger(() -> elevator.isAtTarget());
    }

    public Boolean isAtLocation(Distance position)  {
        return Math.abs(position.in(Meters) - elevator.getPosition()) < 0.02;
    }

    public double getDriveFactor() {
        if (elevator.getPosition() > 1.0) {
            return 0.0;
        } else {
            return (1.0 + LOAD.getLengthInMeters()) - elevator.getPosition();
        }
    }
    public Distance selectElevatorLevel(Supplier<ReefscapeButtonBoard.ScoringLevel> level) {
        switch (level.get()) {
            case L1:
                return L1.getLength();
            case L2:
                return AlgaeArm.algaeSelected.getAsBoolean() ? L2Algae.getLength() : L2.getLength();
            case L3:
                return AlgaeArm.algaeSelected.getAsBoolean() ? L3Algae.getLength() : L3.getLength();
            case L4:
                return L4.getLength();
            default:
                return LOAD.getLength();
        }
    }

    @Override
    public void periodic() {
        elevator.draw();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simulationUpdate();
    }
}
