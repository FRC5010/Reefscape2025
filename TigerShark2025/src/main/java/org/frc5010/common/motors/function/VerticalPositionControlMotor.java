// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.subsystems.PhysicsSim;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class VerticalPositionControlMotor extends GenericControlledMotor {
    protected MechanismLigament2d displayedCarriage;
    protected MechanismLigament2d displayedSetpoint;
    protected MechanismRoot2d mechRoot;
    protected MechanismRoot2d setPointRoot;
    protected ElevatorSim simMechanism;
    protected double setPointDisplayOffset = 0.03;
    Distance carriageHeight = Meters.of(0.0);
    Distance mechanismHeight = Meters.of(0.0);
    Distance minHeight = Meters.of(0.0);
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
        mechanismHeight = maximumHeight;
        simMechanism = new ElevatorSim(
                LinearSystemId.createElevatorSystem(_motor.getMotorSimulationType(), mass.in(Kilograms),
                        drumRadius.in(Meters), gearing),
                _motor.getMotorSimulationType(), minHeight.in(Meters), maximumHeight.in(Meters), true,
                startingHeight.in(Meters));
        double persistedConversion = this.conversionRotationsToDistance.getValue();
        this.kG.setValue(0 == this.kG.getValue() ? kG : this.kG.getValue());
        this.conversionRotationsToDistance
                .setValue(0 == persistedConversion ? (drumRadius.in(Meters) * 2.0 * Math.PI) / gearing
                        : persistedConversion);
        encoder.setPositionConversion(this.conversionRotationsToDistance.getValue());
        encoder.setVelocityConversion(this.conversionRotationsToDistance.getValue() * 60.0);
        encoder.setPosition(startingHeight.in(Meters));
        position.setValue(startingHeight.in(Meters));
        return this;
    }

    @Override
    public VerticalPositionControlMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
        super.setVisualizer(visualizer, robotToMotor);
        MechanismRoot2d mechanismRoot = visualizer.getRoot(
                _visualName + "mechRoot",
                getSimX(Meters.of(robotToMotor.getX() + -setPointDisplayOffset)),
                getSimY(Meters.of(robotToMotor.getZ())));

        MechanismLigament2d mechanismStructure = new MechanismLigament2d(
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

        displayedSetpoint = new MechanismLigament2d(
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
        displayedCarriage = new MechanismLigament2d(
                _visualName + "-carriage",
                carriageHeight.in(Meters),
                90,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        mechRoot.append(displayedCarriage);
        return this;
    }

    public Boolean isAtMax() {
        return isCloseToMax(Meters.of(0));
    }

    public Boolean isAtMin() {
        return isCloseToMin(Meters.of(0));
    }

    public Boolean isCloseToMax(Distance closeZone) {
        return getPosition() >= mechanismHeight.minus(closeZone).in(Meters) ;
    }

    public Boolean isCloseToMin(Distance closeZone) {
        return getPosition() <= minHeight.plus(closeZone).in(Meters);
    }

    @Override
    public void setReference(double reference) {
        setReference(reference, controller.getControlType(),
                getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage());
    }

    public void updateReference() {
        if (PIDControlType.NONE != controller.getControlType()) {
            controller.setReference(reference.getValue(), getControlType(),
                    getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage());
        }
    }

    @Override
    public void set(double speed) {
        double actual = MathUtil.clamp(speed + getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage(), -1.0,
                1.0);
        this.speed.setValue(actual);
        _motor.set(actual);
    }

    public double getPosition() {
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
        if (null == elevatorFeedforward || supplyKG.isPresent() || GenericRobot.logLevel == LogLevel.CONFIG) {
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

    @Override
    public void draw() {
        updateReference();
        double currentPosition = 0;
        effort.setVoltage(_motor.getVoltage(), Volts);
        currentPosition = getPosition();
        setPointRoot.setPosition(getSimX(Meters.of(_robotToMotor.getX())) + setPointDisplayOffset,
                getSimY(Meters.of(_robotToMotor.getZ())) + getReference());
        mechRoot.setPosition(getSimX(Meters.of(_robotToMotor.getX())),
                getSimY(Meters.of(_robotToMotor.getZ())) + currentPosition);
        position.setValue(currentPosition);
        velocity.setValue(encoder.getVelocity());
    }

    @Override
    public void simulationUpdate() {
        simMechanism.setInput(_motor.getVoltage());
        effort.setVoltage(_motor.getVoltage(), Volts);
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
                5,
                3,
                3);
    }

}
