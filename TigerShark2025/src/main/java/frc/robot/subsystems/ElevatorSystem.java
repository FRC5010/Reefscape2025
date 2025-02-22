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
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;
import org.frc5010.common.sensors.ValueSwitch;
import org.frc5010.common.telemetry.DisplayLength;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;

/** Add your docs here. */
public class ElevatorSystem extends GenericSubsystem {
    protected ValueSwitch hasHighCurrentLoad;
    protected VerticalPositionControlMotor elevator;
    protected FollowerMotor elevatorFollower;
    protected PIDControlType controlType = PIDControlType.NONE;
    protected Distance safeDistance = Inches.of(0.5);
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
    // public DisplayLength HowTo = new DisplayLength(Meters.of(0.06), "HowTo",
    // "MyTable");
    private double currentX = 0.0, lastX = 0.0, lastTime = 0.0, timeChange = 0.0, currentVelocity = 0.0,
            lastVelocity = 0.0, currentAcceleration = 0.0;

    private ProfiledPIDController profiledPID;
    private TrapezoidProfile.Constraints PIDConstraints;

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

    public static class Config {
        private final Current MAX_ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(120);
        private final Current MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(60);
        private Distance centerOfMassX = Meters.zero(), centerOfMassY = Inches.of(1.178), wheelBase = Meters.of(0.56);
        private double g = 9.81, growFactor = 0.233035, exponent = 0.824063, initialValue = 0.189682;
        private final double ELEVATOR_ZERO_CURRENT = 40;
        private SlewRateLimiter rateLimiter = new SlewRateLimiter(0.5);
    }

    private Config config = new Config();

    public ElevatorSystem(Mechanism2d mechanismSimulation, Config config) {
        if (config != null) this.config = config;

        if (0 == BOTTOM.getLength().in(Meters))
            BOTTOM.setLength(Position.BOTTOM.position());
        if (0 == LOAD.getLength().in(Meters))
            LOAD.setLength(Position.LOAD.position());
        if (0 == PROCESSOR.getLength().in(Meters))
            PROCESSOR.setLength(Position.PROCESSOR.position());
        if (0 == L1.getLength().in(Meters))
            L1.setLength(Position.L1.position());
        if (0 == L2Algae.getLength().in(Meters))
            L2Algae.setLength(Position.L2Algae.position());
        if (0 == L2Shoot.getLength().in(Meters))
            L2Shoot.setLength(Position.L2Shoot.position());
        if (0 == L2.getLength().in(Meters))
            L2.setLength(Position.L2.position());
        if (0 == L3Algae.getLength().in(Meters))
            L3Algae.setLength(Position.L3Algae.position());
        if (0 == L3Shoot.getLength().in(Meters))
            L3Shoot.setLength(Position.L3Shoot.position());
        if (0 == L3.getLength().in(Meters))
            L3.setLength(Position.L3.position());
        if (0 == L4Shoot.getLength().in(Meters))
            L4Shoot.setLength(Position.L4Shoot.position());
        if (0 == L4.getLength().in(Meters))
            L4.setLength(Position.L4.position());
        if (0 == NET.getLength().in(Meters))
            NET.setLength(Position.NET.position());

        elevator = new VerticalPositionControlMotor(MotorFactory.TalonFX(9, Motor.KrakenX60), "elevator",
                displayValues);
        elevatorFollower = new FollowerMotor(MotorFactory.TalonFX(10, Motor.KrakenX60),
                elevator, "elevatorFollower", true);
        elevator.setControlType(controlType);

        elevator.setMotorBrake(true);
        elevator.setCurrentLimit(config.MAX_ELEVATOR_STATOR_CURRENT_LIMIT);
        ((GenericTalonFXMotor) elevator.getMotorController()).setSupplyCurrent(config.MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT);

        elevatorFollower.setMotorBrake(true);

        elevator.setupSimulatedMotor(6, Pounds.of(30), Inches.of(1.1),
                LOAD.getLength(), Inches.of(83.475 - 6.725), LOAD.getLength(),
                Meters.of(0.2), RobotBase.isSimulation() ? 0.75 : 0.263672);
        elevatorFollower.setCurrentLimit(config.MAX_ELEVATOR_STATOR_CURRENT_LIMIT);
        ((GenericTalonFXMotor) elevatorFollower.getMotorController())
                .setSupplyCurrent(config.MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT);

        elevator.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(5.75).in(Meters), Inches.of(4.75).in(Meters), Inches.of(6.725).in(Meters)),
                new Rotation3d()));

        elevator.setMotorFeedFwd(new MotorFeedFwdConstants(0.26329, 0.38506, 0.04261));
        elevator.setProfiledMaxVelocity(2.5);
        elevator.setProfiledMaxAcceleration(2.5 / 0.25);
        elevator.setValues(new GenericPID(3, 0.0, 0.0));
        elevator.setOutputRange(-1, 1);

        PIDConstraints = new TrapezoidProfile.Constraints(elevator.getProfiledMaxVelocity(),
                elevator.getProfiledMaxAcceleration());
        profiledPID = new ProfiledPIDController(elevator.getP(), elevator.getI(), elevator.getD(), PIDConstraints);
        profiledPID.setTolerance(0.01);

        elevator.getMotorEncoder().setPosition(LOAD.getLengthInMeters());

        // Tell the elevator to run the motor in reverse because the simulator thinks CW
        // is upwards
        if (RobotBase.isSimulation()) {
            elevator.invert(true);
            // Tell the simulator that the motor is CW.
            elevator.getMotorEncoder().setInverted(true);
        }

        elevator.burnFlash();

        hasHighCurrentLoad = new ValueSwitch(config.ELEVATOR_ZERO_CURRENT, () -> Math.abs(elevator.getOutputCurrent()), 1);

        hasHighCurrentLoad.getTrigger().and(() -> elevator.getPosition() < 0.25).and(() -> elevator.get() < -0.1)
                .onTrue(zeroElevator());
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
            speed = speed * 0.13;
            config.rateLimiter.reset(speed);
        } else {
            double currentSpeed = elevator.get();
            if ((Math.signum(speed) > 0 && currentSpeed < speed) ||
                    (Math.signum(speed) < 0 && speed < currentSpeed)) {
                speed = config.rateLimiter.calculate(speed);
            } else {
                config.rateLimiter.reset(speed);
            }
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

    public Command elevatorPositionZeroSequence() {
        double zeroSpeed = -0.1;
        return Commands.run(() -> elevator.set(zeroSpeed), this).until(hasHighCurrentLoad.getTrigger())
                .andThen(zeroElevator());
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
            double output = profiledPID.calculate(elevator.getPosition());
            elevator.set(MathUtil.clamp(output, -1, 1), profiledPID.getSetpoint().velocity);

        }, this).beforeStarting(() -> {
            profiledPID.setPID(elevator.getP(), elevator.getI(), elevator.getD());
            profiledPID.reset(elevator.getPosition(), elevator.getVelocity());
            profiledPID.setGoal(position.in(Meters));
            elevator.setReference(position.in(Meters));
            elevator.setControlType(PIDControlType.NONE);
        }).finallyDo(() -> {
            elevator.setControlType(controlType);
        });
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

    public Command joystickPositionControl(Supplier<Distance> positionalChange) {
        return Commands.run(() -> {
            setElevatorPosition(Meters.of(elevator.getPosition()).plus(positionalChange.get()));
        }, this).beforeStarting(setControlType(PIDControlType.PROFILED_POSITION)).finallyDo(this::resetControlType);
    }

    private Command setControlType(PIDControlType type) {
        return Commands.runOnce(() -> elevator.setControlType(type));
    }

    private void resetControlType() {
        elevator.setControlType(controlType);
    }

    public Trigger isAtTarget() {
        return new Trigger(() -> elevator.isAtTarget());
    }

    public Boolean isAtLocation(Distance position) {
        return Math.abs(position.in(Meters) - elevator.getPosition()) < 0.02;
    }

    public double getCenterOfMassZ() {
        return config.growFactor * Math.pow(elevator.getPosition(), config.exponent) + config.initialValue;
    }

    // Function that decreases acceleration to counteract elevator flex
    public double getAccelerationDampener() {
        return Math.pow(elevator.getPosition(), 2) + 1;
    }

    public double getMaxForwardAcceleration() {
        return ((((config.wheelBase.in(Meters) / 2) + config.centerOfMassY.in(Meters))
                * (config.g + getCOMAcceleration(elevator.getPosition()))) / getCenterOfMassZ()) - getAccelerationDampener();
    }

    public double getMaxBackwardAcceleration() {
        return -((((config.wheelBase.in(Meters) / 2) - config.centerOfMassY.in(Meters))
                * (config.g + getCOMAcceleration(elevator.getPosition()))) / getCenterOfMassZ()) + getAccelerationDampener();
    }

    public double getMaxRightAcceleration() {
        return ((((config.wheelBase.in(Meters) / 2) + config.centerOfMassX.in(Meters))
                * (config.g + getCOMAcceleration(elevator.getPosition()))) / getCenterOfMassZ()) - getAccelerationDampener();
    }

    public double getMaxLeftAcceleration() {
        return -((((config.wheelBase.in(Meters) / 2) - config.centerOfMassX.in(Meters))
                * (config.g + getCOMAcceleration(elevator.getPosition()))) / getCenterOfMassZ()) + getAccelerationDampener();
    }

    public double getCOMAcceleration(double x) {
        timeChange = (RobotController.getFPGATime() - lastTime) * Math.pow(10, 6);
        lastTime = RobotController.getFPGATime();
        currentVelocity = (x - lastX) / timeChange;
        lastX = x;
        currentAcceleration = (currentVelocity - lastVelocity) / timeChange;
        lastVelocity = currentVelocity;
        return (config.growFactor * config.exponent * (config.exponent - 1) * Math.pow(x, config.exponent - 2) * Math.pow(currentVelocity, 2))
                + (config.growFactor * config.exponent * Math.pow(x, config.exponent - 1) * currentAcceleration);
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
        SmartDashboard.putNumber("forward acceleration", getMaxForwardAcceleration());
        SmartDashboard.putNumber("backward acceleration", getMaxBackwardAcceleration());
        SmartDashboard.putNumber("left acceleration", getMaxLeftAcceleration());
        SmartDashboard.putNumber("right acceleration", getMaxRightAcceleration());
        SmartDashboard.putNumber("Center of Mass Z", getCenterOfMassZ());
    }

    @Override
    public void simulationPeriodic() {
        elevator.simulationUpdate();
    }
}
