// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.motors.MotorController5010;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Configuration for a YamsTurret
 */
public class YamsPivotConfigurationJson implements DeviceConfiguration {
    public MotorSetupJson motorSetup = new MotorSetupJson();
    public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
    public UnitValueJson lowerHardLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson upperHardLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson startingAngle = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson lowerSoftLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson upperSoftLimit = new UnitValueJson(0, UnitsParser.DEG);
    public double[] gearing;
    public UnitValueJson voltageCompensation = new UnitValueJson(12, UnitsParser.VOLTS);
    public UnitValueJson startingPosition = new UnitValueJson(0, UnitsParser.DEG);
    public double moi;

    /**
     * Configure the given GenericSubsystem with a pivot using the given json
     * configuration.
     *
     * @param deviceHandler the GenericSubsystem to configure
     * @return the configured pivot
     */
    @Override
    public Pivot configure(SubsystemBase deviceHandler) {
        MotorController5010 motor = DeviceConfigReader.getMotor(motorSetup.controllerType, motorSetup.motorType,
                motorSetup.canId);

        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(deviceHandler)
                .withClosedLoopController(motorSystemId.feedBack.p, motorSystemId.feedBack.i, motorSystemId.feedBack.d,
                        UnitsParser.parseAngularVelocity(motorSystemId.maxVelocity),
                        UnitsParser.parseAngularAcceleration(motorSystemId.maxAcceleration))
                .withSoftLimit(UnitsParser.parseAngle(lowerSoftLimit),
                        UnitsParser.parseAngle(upperSoftLimit))
                .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(gearing)))
                .withIdleMode(MotorMode.valueOf(motorSetup.idleMode))
                .withTelemetry(motorSetup.name + "Motor", TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withStatorCurrentLimit(UnitsParser.parseAmps(motorSetup.currentLimit))
                .withMotorInverted(motorSetup.inverted)
                .withClosedLoopRampRate(UnitsParser.parseTime(motorSystemId.closedLoopRamp))
                .withOpenLoopRampRate(UnitsParser.parseTime(motorSystemId.openLoopRamp))
                .withFeedforward(new ArmFeedforward(motorSystemId.feedForward.s, motorSystemId.feedForward.g,
                        motorSystemId.feedForward.v, motorSystemId.feedForward.a))
                .withControlMode(ControlMode.valueOf(motorSystemId.controlMode));
        MotorSetupJson.setupFollowers(motorConfig, motorSetup);
        motor.setMotorSimulationType(motor.getMotorConfig().getMotorSimulationType(motorSetup.numberOfMotors));

        SmartMotorController smartMotor = motor.getSmartMotorController(motorConfig);
        PivotConfig pivotConfig = new PivotConfig(smartMotor)
                .withHardLimit(UnitsParser.parseAngle(lowerHardLimit),
                        UnitsParser.parseAngle(upperHardLimit))
                .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withStartingPosition(UnitsParser.parseAngle(startingPosition))
                .withMOI(moi);
        Pivot pivot = new Pivot(pivotConfig);
        return pivot;
    }
}
