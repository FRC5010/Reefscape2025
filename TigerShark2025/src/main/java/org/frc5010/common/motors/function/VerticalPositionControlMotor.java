// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.subsystems.PhysicsSim;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/** Add your docs here. */
public class VerticalPositionControlMotor extends GenericControlledMotor {
    protected LoggedMechanismLigament2d displayedCarriage;
    protected LoggedMechanismLigament2d displayedSetpoint;
    protected LoggedMechanismRoot2d mechRoot;
    protected LoggedMechanismRoot2d setPointRoot;
    protected ElevatorSim simMechanism;
    protected double setPointDisplayOffset = 0.03;
    Distance carriageHeight = Meters.of(0.0);
    Distance mechanismHeight = Meters.of(0.0);
    Distance minHeight = Meters.of(0.0);
    Distance maximumHeight = Meters.of(0.0);
    Distance startingHeight = Meters.of(0.0);
    protected final String K_G = "kG";
    protected final String CONVERSION = "Conversion";
    protected final String SPEED = "Speed";
    protected DisplayDouble kG;
    protected DisplayDouble conversionRotationsToDistance;
    protected DisplayDouble speed;
    protected Optional<DoubleSupplier> supplyKG = Optional.empty();
    protected ElevatorFeedforward elevatorFeedforward;

    public VerticalPositionControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
        super(motor, visualName, tab);
        kG = _displayValuesHelper.makeConfigDouble(K_G);
        this.conversionRotationsToDistance = _displayValuesHelper.makeConfigDouble(CONVERSION);
        this.speed = _displayValuesHelper.makeDisplayDouble(SPEED);
        setControlType(PIDControlType.DUTY_CYCLE);
    }

    public VerticalPositionControlMotor setupSimulatedMotor(double gearing, Mass mass, Distance drumRadius,
            Distance minHeight,
            Distance maximumHeight, Distance startingHeight, Distance carriageHeight, double kG) {
        this.carriageHeight = carriageHeight;
        this.minHeight = minHeight;
        this.maximumHeight = maximumHeight;
        this.startingHeight = startingHeight;
        mechanismHeight = maximumHeight;
        simMechanism = new ElevatorSim(
                LinearSystemId.createElevatorSystem(_motor.getMotorSimulationType(), mass.in(Kilograms),
                        drumRadius.in(Meters), gearing),
                _motor.getMotorSimulationType(), minHeight.in(Meters), maximumHeight.in(Meters), true,
                startingHeight.in(Meters));
        this.kG.setValue(0 == this.kG.getValue() ? kG : this.kG.getValue());
        this.conversionRotationsToDistance
                .setValue((drumRadius.in(Meters) * 2.0 * Math.PI) / gearing);
        double persistedConversion = this.conversionRotationsToDistance.getValue();
        encoder.setPositionConversion(persistedConversion);
        encoder.setVelocityConversion(persistedConversion / 60.0);
        encoder.setPosition(startingHeight.in(Meters));
        position.setValue(startingHeight.in(Meters));
        return this;
    }

    @Override
    public VerticalPositionControlMotor setVisualizer(LoggedMechanism2d visualizer, Pose3d robotToMotor) {
        super.setVisualizer(visualizer, robotToMotor);
        LoggedMechanismRoot2d mechanismRoot = visualizer.getRoot(
                _visualName + "mechRoot",
                getSimX(Meters.of(robotToMotor.getX() + -setPointDisplayOffset)),
                getSimY(Meters.of(robotToMotor.getZ())));

        LoggedMechanismLigament2d mechanismStructure = new LoggedMechanismLigament2d(
                _visualName + "-mechanism",
                mechanismHeight.in(Meters),
                90,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        mechanismRoot.append(mechanismStructure);

        setPointRoot = visualizer.getRoot(
                _visualName + "setRoot",
                getSimX(Meters.of(robotToMotor.getX() + setPointDisplayOffset)),
                getSimY(Meters.of(robotToMotor.getZ())));

        displayedSetpoint = new LoggedMechanismLigament2d(
                _visualName + "-setpoint",
                carriageHeight.in(Meters),
                90,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        setPointRoot.append(displayedSetpoint);

        mechRoot = visualizer.getRoot(
                _visualName,
                getSimX(Meters.of(robotToMotor.getX())),
                getSimY(Meters.of(robotToMotor.getZ())));
        displayedCarriage = new LoggedMechanismLigament2d(
                _visualName + "-carriage",
                carriageHeight.in(Meters),
                90,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        mechRoot.append(displayedCarriage);
        return this;
    }

    public Boolean isAtMax() {
        return isCloseToMax(Meters.of(0.01));
    }

    public MotorController5010 getMotorController() {
        return _motor;
    }

    public Boolean isAtMin() {
        return isCloseToMin(Meters.of(0.01));
    }

    public Boolean isCloseToMax(Distance closeZone) {
        return position.getValue() >= mechanismHeight.minus(closeZone).in(Meters);
    }

    public Boolean isCloseToMin(Distance closeZone) {
        return position.getValue() <= minHeight.plus(closeZone).in(Meters);
    }

    @Override
    public void setReference(double reference) {
        setReference(reference, controller.getControlType(),
                getFeedForward(0.0001 * Math.signum(reference - position.getValue())).in(Volts));
    }

    public void updateReference() {
        if (PIDControlType.NONE != controller.getControlType()) {
            controller.setReference(reference.getValue(), getControlType(),
                    getFeedForward(0.0001 * Math.signum(reference.getValue() - position.getValue())).in(Volts));
        }
    }

    @Override
    public void set(double speed) {

        double actual = MathUtil.clamp(
                speed + getDirectionalFeedForward((int) Math.signum(speed)).in(Volts)
                        / RobotController.getBatteryVoltage(),
                -1.0, 1.0);
        this.speed.setValue(actual);
        outputEffort.setVoltage(actual * outputFactor.getValue(), Volts);
        _motor.set(actual);
    }

    public double getEncoderFeedback() {
        if (RobotBase.isReal()) {
            return encoder.getPosition();
        } else {
            return simMechanism.getPositionMeters();
        }
    }

    public void setSupplyKG(DoubleSupplier supplyKG) {
        this.supplyKG = Optional.of(supplyKG);
    }

    @Override
    public Voltage getFeedForward(double velocity) {
        if (supplyKG.isPresent()) {
            kG.setValue(supplyKG.get().getAsDouble());
        }
        if (null == elevatorFeedforward || supplyKG.isPresent() || _displayValuesHelper.getLoggingLevel() == LogLevel.CONFIG) {
            elevatorFeedforward = new ElevatorFeedforward(
                    kS.getValue(),
                    kG.getValue(),
                    kV.getValue(),
                    kA.getValue());
        }
        Voltage ff = Volts.of(
                elevatorFeedforward.calculate(velocity));
        feedForward.setValue(ff.in(Volts));
        return ff;
    }

    public Voltage getDirectionalFeedForward(int movementDirection) {
        if (supplyKG.isPresent()) {
            kG.setValue(supplyKG.get().getAsDouble());
        }
        if (null == elevatorFeedforward || supplyKG.isPresent() || _displayValuesHelper.getLoggingLevel() == LogLevel.CONFIG) {
            elevatorFeedforward = new ElevatorFeedforward(
                    kS.getValue(),
                    kG.getValue(),
                    kV.getValue(),
                    kA.getValue());
        }
        Voltage ff = Volts
                .of(elevatorFeedforward.getKs() * Math.signum(movementDirection) + elevatorFeedforward.getKg());
        return ff;
    }

    @Override
    public void periodicUpdate() {
        updateReference();
        double currentPosition = 0;
        currentPosition = getEncoderFeedback();
        setPointRoot.setPosition(getSimX(Meters.of(_robotToMotor.getX())) + setPointDisplayOffset,
                getSimY(Meters.of(_robotToMotor.getZ())) + getReference());
        mechRoot.setPosition(getSimX(Meters.of(_robotToMotor.getX())),
                getSimY(Meters.of(_robotToMotor.getZ())) + currentPosition);
        position.setValue(currentPosition);
        velocity.setValue(encoder.getVelocity());
        encoderFeedback.setValue(currentPosition);

        if (Robot.isSimulation()) {
            actualEffort.setVoltage(_motor.getVoltage(), Volts);
        }
    }

    @Override
    public void simulationUpdate() {
        simMechanism.setInput(_motor.getVoltage());
        outputEffort.setVoltage(_motor.getVoltage(), Volts);
        simMechanism.update(PhysicsSim.SimProfile.getPeriod());
        _motor.simulationUpdate(Optional.of(simMechanism.getPositionMeters()),
                simMechanism.getVelocityMetersPerSecond());

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(simMechanism.getCurrentDrawAmps()));
    }

    @Override
    public Command getSysIdCommand(SubsystemBase subsystemBase) {
        return SystemIdentification.getSysIdFullCommand(
                SystemIdentification.angleSysIdRoutine(_motor, encoder, "Vertical Motor", subsystemBase),
                4,
                1,
                1);
    }

}
