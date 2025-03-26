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
    private double forwardA = 0.0, forwardB = 0.0, forwardC = 0.0, backwardA = 0.0, backwardB = 0.0, backwardC = 0.0, horizontalA = 0.0, horizontalB = 0.0, horizontalC = 0.0, elevatorA = 0.0, elevatorB = 0.0, elevatorC = 0.0;

    private ProfiledPIDController profiledPID;
    private TrapezoidProfile.Constraints PIDConstraints;

    public static enum Position {
        BOTTOM(Meters.of(0.0)),
        LOAD(Meters.of(0.09)),
        CORAL_EXTENSION(Meters.of(0.30)),
        PROCESSOR(Meters.of(0.15)),
        L1(Meters.of(0.72)),
        L2Algae(Meters.of(0.11)),
        L2Shoot(Meters.of(0.87)),
        L2(Meters.of(0.8579)),
        L3Algae(Meters.of(1.1)),
        L3Shoot(Meters.of(1.27)),
        L3(Meters.of(1.25)),
        L4Shoot(Meters.of(1.765)),
        L4(Meters.of(1.87)),
        NET(Meters.of(1.9));

        private final Distance position;

        private Position(Distance position) {
            this.position = position;
        }

        public Distance position() {
            return position;
        }
    }

    private Distance centerOfMassX = Meters.zero(), centerOfMassY = Inches.of(1.178), wheelBase = Meters.of(0.56);
    private double growFactor = 0.233035, exponent = 0.824063, initialValue = 0.189682;
    private double lastError;
    private double lastTimestamp = 0;
    private int elevatorAtReferenceCounter = 0;
    public Supplier<Distance> stoppingDistance = () -> getStoppingDistance();

    public static class Config {
        public final Current MAX_ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(120);
        public final Current MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT = Amps.of(60);
        public Distance centerOfMassX = Meters.zero(), centerOfMassY = Inches.of(1.178), wheelBase = Meters.of(0.56);
        public double g = 9.81;
        public final double ELEVATOR_ZERO_CURRENT = 40;
        public SlewRateLimiter rateLimiter = new SlewRateLimiter(0.5);
        public double gearing = 6;
    }

    private Config config = new Config();

    public ElevatorSystem(Mechanism2d mechanismSimulation, Config config) {
        if (config != null)
            this.config = config;

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
        ((GenericTalonFXMotor) elevator.getMotorController())
                .setSupplyCurrent(config.MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT);

        elevatorFollower.setMotorBrake(true);

        elevator.setupSimulatedMotor(config.gearing, Pounds.of(30), Inches.of(1.1),
                LOAD.getLength(), Inches.of(83.475 - 6.725), LOAD.getLength(),
                Meters.of(0.2), RobotBase.isSimulation() ? 0.75 : 0.263672);
        elevatorFollower.setCurrentLimit(config.MAX_ELEVATOR_STATOR_CURRENT_LIMIT);
        ((GenericTalonFXMotor) elevatorFollower.getMotorController())
                .setSupplyCurrent(config.MAX_ELEVATOR_SUPPLY_CURRENT_LIMIT);

        elevator.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(5.75).in(Meters), Inches.of(4.75).in(Meters), Inches.of(6.725).in(Meters)),
                new Rotation3d()));

        elevator.setMotorFeedFwd(new MotorFeedFwdConstants(0.26329, 0.38506, 0.04261));
        elevator.setProfiledMaxVelocity(5.5);
        elevator.setProfiledMaxAcceleration(16);
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

        hasHighCurrentLoad = new ValueSwitch(config.ELEVATOR_ZERO_CURRENT, () -> Math.abs(elevator.getOutputCurrent()),
                1);

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
                .andThen(zeroElevator()).finallyDo(() -> elevator.set(0.0));
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
            runControllerToSetpoint();

        }, this).beforeStarting(() -> {
            resetController(position);
            elevator.setControlType(PIDControlType.NONE);
        }).finallyDo(() -> {
            elevator.set(0);
            elevator.setControlType(controlType);
        });
    }

    public Command stopElevator() {
        return Commands.run(() -> {
            elevator.set(0);
            elevator.setControlType(controlType);
        });
    }

    public void runControllerToSetpoint() {
        double output = profiledPID.calculate(elevator.getPosition());
        elevator.set(MathUtil.clamp(output, -1, 1), profiledPID.getSetpoint().velocity);
    }

    public void setControllerGoal(Distance goal) {
        profiledPID.setGoal(goal.in(Meters));
        elevator.setReference(goal.in(Meters));
    }

    public void resetController(Distance goal) {
        profiledPID.reset(elevator.getPosition(), elevator.getVelocity());
        profiledPID.setPID(elevator.getP(), elevator.getI(), elevator.getD());
        setControllerGoal(goal);
    }

    private void setElevatorPIDGoal(double goal) {
        profiledPID.setGoal(goal);
        elevator.setReference(goal);
    }

    public Command pidControlCommand(Distance position, Supplier<Distance> maxHeight) {
        return Commands.run(() -> {
            if (profiledPID.getGoal().position != position.in(Meters)) {
                setElevatorPIDGoal(Math.min(maxHeight.get().in(Meters), position.in(Meters)));
            }
            double output = profiledPID.calculate(elevator.getPosition());
            elevator.set(MathUtil.clamp(output, -1, 1), profiledPID.getSetpoint().velocity);

        }, this).beforeStarting(() -> {
            profiledPID.setPID(elevator.getP(), elevator.getI(), elevator.getD());
            profiledPID.reset(elevator.getPosition(), elevator.getVelocity());
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

    public double getElevatorReference() {
        return elevator.getReference();
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

    public boolean isAtLocation(Distance position) {
        return Math.abs(position.in(Meters) - elevator.getPosition()) < 0.02;
    }

    public boolean isAtLocationImproved(Distance position) {
        if (Math.abs(position.in(Meters) - elevator.getPosition()) < 0.02) {
            elevatorAtReferenceCounter++;
            return elevatorAtReferenceCounter > 10;
        }
        elevatorAtReferenceCounter = 0;
        return false;
    }

    public double getCenterOfMassZ() {
        return growFactor * Math.pow(elevator.getPosition(), exponent) + initialValue;
    }

    // Function that decreases acceleration to counteract elevator flex
    public double getGeneralAccelerationDampener() {
        return Math.pow(elevator.getPosition()*2, 0.5);
    }

    public double getBackwardAccelerationDampener() {
        return Math.pow(elevator.getPosition()*2, 0.5);
    }

    public double getMaxForwardAcceleration() {
        return ((((wheelBase.in(Meters) / 2) + centerOfMassY.in(Meters))
                * (config.g)) / getCenterOfMassZ())
                - getGeneralAccelerationDampener();
    }

    public double getMaxForwardVelocity() {
        return Math.sqrt(2 * (Math.abs(getMaxBackwardAcceleration())) * stoppingDistance.get().in(Meters));
    }

    public double getMaxBackwardAcceleration() {
        return -((((wheelBase.in(Meters) / 2) - centerOfMassY.in(Meters))
                * (config.g)) / getCenterOfMassZ())
                + getBackwardAccelerationDampener();
    }

    public double getMaxBackwardVelocity() {
        return Math.sqrt(2 * (-Math.abs(getMaxForwardAcceleration())) * stoppingDistance.get().in(Meters));
    }

    public double getMaxRightAcceleration() {
        return ((((wheelBase.in(Meters) / 2) + centerOfMassX.in(Meters))
                * (config.g)) / getCenterOfMassZ())
                - getGeneralAccelerationDampener();
    }

    public double getMaxRightVelocity() {
        return Math.sqrt(2 * (Math.abs(getMaxLeftAcceleration())) * stoppingDistance.get().in(Meters));
    }

    public double getMaxLeftAcceleration() {
        return -((((wheelBase.in(Meters) / 2) - centerOfMassX.in(Meters))
                * (config.g)) / getCenterOfMassZ())
                + getGeneralAccelerationDampener();
    }

    public double getMaxLeftVelocity() {
        return Math.sqrt(2 * (-Math.abs(getMaxRightAcceleration())) * stoppingDistance.get().in(Meters));
    }

    public double getCOMAcceleration(double x) {
        timeChange = (RobotController.getFPGATime() - lastTime) / 1E6;
        lastTime = RobotController.getFPGATime();
        currentVelocity = MathUtil.clamp((x - lastX) / timeChange, -2.5, 2.5); // Note: temporary fix, add data
                                                                               // smoothing later
        lastX = x;
        currentAcceleration = MathUtil.clamp((currentVelocity - lastVelocity) / timeChange, 2.5, -2.5); // Note:
                                                                                                        // temporary
                                                                                                        // fix, add data
                                                                                                        // smoothing
                                                                                                        // later
        lastVelocity = currentVelocity;
        return (growFactor * exponent * (exponent - 1) * Math.pow(x, exponent - 2)
                * Math.pow(currentVelocity, 2))
                + (growFactor * exponent * Math.pow(x, exponent - 1) * currentAcceleration);
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

    public ProfiledPIDController getPIDController() {
        return profiledPID;
    }

    // returns constants [+acceleration, -+time (time to accelerate in one
    // direction), time]
    public double[] getHeightTimeFunction(double height) {
        double distance = elevator.getPosition() - height;
        double timeToMaxVelocity = elevator.getProfiledMaxVelocity() / elevator.getProfiledMaxAcceleration();
        double maxTriangleDistance = elevator.getProfiledMaxAcceleration() * Math.pow(timeToMaxVelocity, 2);
        double accelerationTime = 0.0, time = 0.0;
        if (maxTriangleDistance > height) {
            accelerationTime = Math.sqrt(distance / elevator.getProfiledMaxAcceleration());
            time = 0.0;
        } else {
            double remainingDistance = distance - maxTriangleDistance;
            accelerationTime = Math.sqrt(maxTriangleDistance / elevator.getProfiledMaxAcceleration());
            time = remainingDistance / elevator.getProfiledMaxVelocity();
        }
        return new double[] { elevator.getProfiledMaxAcceleration(), accelerationTime, time };
    }

    public void setRobotParameters(Distance centerofMassX, Distance centerOfMassY, Distance wheelBase,
            double growFactor, double exponent, double initialValue) {
        this.centerOfMassX = centerOfMassX;
        this.centerOfMassY = centerOfMassY;
        this.wheelBase = wheelBase;
        this.growFactor = growFactor;
        this.exponent = exponent;
        this.initialValue = initialValue;
    }

    public void setCOGFunctionParameters(double growFactor, double exponent, double intialValue) {
        this.growFactor = growFactor;
        this.exponent = exponent;
        this.initialValue = intialValue;
    }

    public boolean atLoading() {
        return Math.abs(elevator.getPosition() - Position.LOAD.position().in(Meters)) < 0.06;
    }

    public Distance getElevatorPosition() {
        return Meters.of(elevator.getPosition());
    }

    public Distance getStoppingDistance() {
        if (Math.abs(getElevatorPosition().in(Meters) - Position.L4.position().in(Meters)) < 0.05) {
            return Meters.of(0.3);
        } else {
            return Meters.of(1.0);
        }
    }

    public void setUpElevatorVelocityFunction(double elevatorA, double elevatorB, double elevatorC) {
        this.elevatorA = elevatorA;
        this.elevatorB = elevatorB;
        this.elevatorC = elevatorC;
    }

    public void setUpAccelerationConstraints(double forwardA, double forwardB, double forwardC, double backwardA, double backwardB, double backwardC, double horizontalA, double horizontalB, double horizontalC) {
        this.forwardA = forwardA;
        this.forwardB = forwardB;
        this.forwardC = forwardC;
        this.backwardA = backwardA;
        this.backwardB = backwardB;
        this.backwardC = backwardC;
        this.horizontalA = horizontalA;
        this.horizontalB = horizontalB;
        this.horizontalC = horizontalC;
    }

    public double[] getForwardAccelerationConstants() {
        return new double[] {forwardA, forwardB, forwardC};
    }

    public double[] getBackwardAccelerationConstants() {
        return new double[] {backwardA, backwardB, backwardC};
    }

    public double[] getHorizontalAccelerationConstants() {
        return new double[] {horizontalA, horizontalB, horizontalC};
    }

    public double getElevatorExtensionTime() {
        return elevatorA * Math.pow(elevator.getReference() - elevator.getPosition(), elevatorB) + elevatorC;
    }

    public double getAverageElevatorVelocity() {
        return (elevator.getReference() - getElevatorPosition().in(Meters)) / getElevatorExtensionTime();
    }

    @Override
    public void periodic() {
        elevator.draw();

        SmartDashboard.putNumber("Elevator Current", Math.abs(elevator.getOutputCurrent()));
        SmartDashboard.putNumber("forward acceleration", getMaxForwardAcceleration());
        SmartDashboard.putNumber("backward acceleration", getMaxBackwardAcceleration());
        SmartDashboard.putNumber("left acceleration", getMaxLeftAcceleration());
        SmartDashboard.putNumber("right acceleration", getMaxRightAcceleration());
        SmartDashboard.putNumber("Center of Mass Z", getCenterOfMassZ());
        SmartDashboard.putNumber("Elevator Position Setpoint", profiledPID.getSetpoint().position);
        SmartDashboard.putNumber("Elevator Velocity Setpoint", profiledPID.getSetpoint().velocity);

        
    }

    @Override
    public void simulationPeriodic() {
        elevator.simulationUpdate();
    }
}
