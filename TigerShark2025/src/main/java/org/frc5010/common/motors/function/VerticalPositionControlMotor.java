// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
    protected MechanismLigament2d simulatedCarriage;
    protected MechanismLigament2d simSetpoint;
    protected MechanismRoot2d root;
    protected ElevatorSim simMotor;
    protected SimulatedEncoder simEncoder;
    Distance carriageHeight = Meters.of(0.0);
    protected final String K_G = "kG";
    protected DisplayDouble kG;

    public VerticalPositionControlMotor(MotorController5010 motor, String visualName, DisplayValuesHelper tab) {
        super(motor, visualName, tab);
        kG = _displayValuesHelper.makeDisplayDouble(K_G);
        setControlType(PIDControlType.POSITION);
    }

    public VerticalPositionControlMotor setupSimulatedMotor(double gearing, Mass mass, Distance drumRadius,
            Distance minHeight,
            Distance maximumHeight, Distance startingHeight, Distance carriageHeight, double kG, double conversion) {
        this.carriageHeight = carriageHeight;
        this.kG.setValue(kG);
        simMotor = new ElevatorSim(
                LinearSystemId.createElevatorSystem(_motor.getMotorSimulationType(), mass.in(Kilograms),
                        drumRadius.in(Meters), gearing),
                _motor.getMotorSimulationType(), minHeight.in(Meters), maximumHeight.in(Meters), true,
                startingHeight.in(Meters));
        simEncoder = new SimulatedEncoder(MotorFactory.getNextSimEncoderPort(), MotorFactory.getNextSimEncoderPort());
        simEncoder.setPositionConversion(conversion);
        simEncoder.setPosition(startingHeight.in(Meters));
        position.setValue(startingHeight.in(Meters));
        return this;
    }

    @Override
    public VerticalPositionControlMotor setVisualizer(Mechanism2d visualizer, Pose3d robotToMotor) {
        super.setVisualizer(visualizer, robotToMotor);
        root = visualizer.getRoot(
                _visualName,
                getSimX(Meters.of(robotToMotor.getX())),
                getSimY(Meters.of(robotToMotor.getZ())));

        simulatedCarriage = new MechanismLigament2d(
                _visualName + "-carriage",
                carriageHeight.in(Meters),
                0,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        simSetpoint = new MechanismLigament2d(
                _visualName + "-setpoint",
                carriageHeight.in(Meters),
                0,
                5,
                new Color8Bit(MotorFactory.getNextVisualColor()));
        root.append(simulatedCarriage);
        root.append(simSetpoint);
        return this;
    }

    @Override
    public void setReference(double reference) {
        super.setReference(reference);
        if (!RobotBase.isReal()) {
            effort.setVoltage(calculateControlEffort(simEncoder.getPosition()), Volts);
            _motor.set(effort.getVoltage().in(Volts) / RobotController.getBatteryVoltage());
        } else {
            pid.setF(getFeedForward().in(Volts) / RobotController.getBatteryVoltage());
        }
    }

    public double getPosition() {
        if (RobotBase.isReal()) {
            return encoder.getPosition();
        } else {
            return simEncoder.getPosition();
        }
    }

    @Override
    public Voltage getFeedForward() {
        ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
                getMotorFeedFwd().getkS(),
                kG.getValue(),
                getMotorFeedFwd().getkV(),
                getMotorFeedFwd().getkA());
        Voltage ff = Volts.of(
                elevatorFeedforward.calculate(getPosition()));
        feedForward.setValue(ff.in(Volts));
        return ff;
    }

    @Override
    public void draw() {
        double currentPosition = 0;
        if (RobotBase.isReal()) {
            currentPosition = encoder.getPosition();
        } else {
            currentPosition = simEncoder.getPosition();
        }
        simSetpoint.setLength(getReference());
        simulatedCarriage.setLength(currentPosition);
        position.setValue(currentPosition);
        reference.setValue(getReference());
    }

    @Override
    public void simulationUpdate() {
        simMotor.setInput(_motor.get() * RobotController.getBatteryVoltage());
        simMotor.update(0.020);
        simEncoder.setPosition(Units.radiansToDegrees(simMotor.getPositionMeters()));

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(simMotor.getCurrentDrawAmps()));
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
