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

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.function.VerticalPositionControlMotor;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;

/** Add your docs here. */
public class ElevatorSystem extends GenericSubsystem {
    protected VerticalPositionControlMotor elevator;
    protected FollowerMotor elevatorFollower;
    protected PIDControlType controlType = PIDControlType.POSITION;
    protected Distance safeDistance = Inches.of(10);

    public static enum Position {
        BOTTOM(Meters.of(0)),
        LOAD(Meters.of(0.05)),
        L1(Meters.of(0.65)),
        L2Algae(Meters.of(0.11)),
        L2Shoot(Meters.of(0.45)),
        L2(Meters.of(0.81)),
        L3Algae(Meters.of(1.1)),
        L3(Meters.of(1.19)),
        L4(Meters.of(1.82)),
        NET(Meters.of(1.9));

        private final Distance position;

        private Position(Distance position) {
            this.position = position;
        }

        public Distance position() {
            return position;
        }
    }

    public ElevatorSystem(Mechanism2d mechanismSimulation) {

        elevator = new VerticalPositionControlMotor(MotorFactory.TalonFX(9, Motor.KrakenX60), "elevator",
                displayValues);
        elevatorFollower = new FollowerMotor(MotorFactory.TalonFX(10, Motor.KrakenX60),
        elevator, "elevatorFollower", true);
        
        

        elevator.setupSimulatedMotor(6, Pounds.of(15), Inches.of(1.1), Meters.of(0), Inches.of(83.475 - 6.725),
                Meters.of(0),
                Meters.of(0.2), 0.263672);
        elevator.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(5.75).in(Meters), Inches.of(4.75).in(Meters), Inches.of(6.725).in(Meters)),
                new Rotation3d()));
        elevator.setCurrentLimit(Amps.of(0));
        elevator.setMotorFeedFwd(new MotorFeedFwdConstants(0.0, 0.0, 0.0));
        elevator.setProfiledMaxVelocity(2.0);
        elevator.setProfiledMaxAcceleration(5);
        elevator.setValues(new GenericPID(0, 0, 0));
        elevator.setOutputRange(-1, 1);
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

    public double safeSpeed(double speed) {
        if (!validSpeed(speed)) {
            return 0.0;
        }
        if ((speed > 0 && elevator.isCloseToMax(safeDistance)) || (speed < 0 && elevator.isCloseToMin(safeDistance))) {
            return 0.1 * speed;
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

    public Command driveToPosition(Position position) {
        return Commands.run(() -> {
            double difference = position.position().in(Meters) - elevator.getPosition();
            double sign = Math.signum(difference);
            double effort = 0.5;
            if (Math.abs(difference) < 0.1) {
                effort = 0.1;

                if (Math.abs(difference) < 0.01) {
                    effort = 0;
                }
            }
            effort *= sign;
            elevator.set(effort);
        }, this);
    }

    public void setElevatorPosition(Position position) {
        if (elevator.getControlType() != controlType) {
            elevator.setControlType(controlType);
        }
        if (controlType == elevator.getControlType()) {
            elevator.setReference(position.position().in(Meters));
        }
    }

    public Command basicSuppliersMovement(DoubleSupplier speed) {
        return Commands.run(
            () -> elevatorSpeed(speed.getAsDouble()), this);

    }

    public Trigger isAtTarget() {
        return new Trigger(() -> elevator.isAtTarget());
    }

    public Position selectElevatorLevel(Supplier<ReefscapeButtonBoard.ScoringLevel> level) {
        switch (level.get()) {
            case L1:
                return Position.L1;
            case L2:
                return Position.L2;
            case L3:
                return Position.L3;
            case L4:
                return Position.L4;
            default:
                return Position.BOTTOM;
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
