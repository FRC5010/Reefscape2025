package org.frc5010.common.config.json.devices;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.motors.MotorController5010;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class YamsElevatorConfigurationJson implements DeviceConfiguration {
    public MotorSetupJson motorSetup = new MotorSetupJson();
    public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
    public int sprocketTeeth = 0;
    public UnitValueJson drumRadius = new UnitValueJson(0, UnitsParser.IN);
    public UnitValueJson lowerSoftLimit = new UnitValueJson(0, UnitsParser.M);
    public UnitValueJson upperSoftLimit = new UnitValueJson(0, UnitsParser.M);
    public UnitValueJson lowerHardLimit = new UnitValueJson(0, UnitsParser.M);
    public UnitValueJson upperHardLimit = new UnitValueJson(0, UnitsParser.M);
    public double[] gearing;
    public UnitValueJson startingPosition = new UnitValueJson(0, UnitsParser.M);
    public UnitValueJson mass = new UnitValueJson(0, UnitsParser.LBS);
    public UnitValueJson voltageCompensation = new UnitValueJson(12, UnitsParser.VOLTS);

    /**
     * Configure the given GenericSubsystem with an elevator using the given json
     * configuration.
     * 
     * @param deviceHandler the GenericSubsystem to configure
     * @return the configured elevator
     */
    @Override
    public Elevator configure(SubsystemBase deviceHandler) {
        MotorController5010 motor = DeviceConfigReader.getMotor(motorSetup.controllerType, motorSetup.motorType,
                motorSetup.canId);
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(deviceHandler);
        if (sprocketTeeth > 0) {
            motorConfig.withMechanismCircumference(
                    Meters.of(Inches.of(0.25).in(Meters) * sprocketTeeth));
        } else if (drumRadius.val > 0) {
            motorConfig.withMechanismCircumference(
                    UnitsParser.parseDistance(drumRadius));
        } else {
            throw new RuntimeException("Must specify either sproketTeeth or drumRadius");
        }

        motorConfig
                .withClosedLoopController(motorSystemId.feedBack.p, motorSystemId.feedBack.i, motorSystemId.feedBack.d,
                        UnitsParser.parseVelocity(motorSystemId.maxVelocity),
                        UnitsParser.parseAccelleration(motorSystemId.maxAcceleration))
                .withSoftLimit(UnitsParser.parseDistance(lowerSoftLimit),
                        UnitsParser.parseDistance(upperSoftLimit))
                .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(gearing)))
                .withIdleMode(MotorMode.valueOf(motorSetup.idleMode))
                .withTelemetry(motorSetup.name + "Motor", TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withStatorCurrentLimit(UnitsParser.parseAmps(motorSetup.currentLimit))
                .withMotorInverted(motorSetup.inverted)
                .withClosedLoopRampRate(
                        UnitsParser.parseTime(motorSystemId.closedLoopRamp))
                .withOpenLoopRampRate(
                        UnitsParser.parseTime(motorSystemId.openLoopRamp))
                .withFeedforward(new ElevatorFeedforward(motorSystemId.feedForward.s, motorSystemId.feedForward.g,
                        motorSystemId.feedForward.v, motorSystemId.feedForward.a))
                .withControlMode(ControlMode.valueOf(motorSystemId.controlMode));
        MotorSetupJson.setupFollowers(motorConfig, motorSetup);
        motor.setMotorSimulationType(motor.getMotorConfig().getMotorSimulationType(motorSetup.numberOfMotors));

        SmartMotorController smartMotor = motor.getSmartMotorController(motorConfig);
        ElevatorConfig m_config = new ElevatorConfig(smartMotor)
                .withStartingHeight(UnitsParser.parseDistance(startingPosition))
                .withHardLimits(UnitsParser.parseDistance(lowerHardLimit), UnitsParser.parseDistance(upperHardLimit))
                .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withMass(UnitsParser.parseMass(mass));
        Elevator elevator = new Elevator(m_config);
        return elevator;
    }
}
