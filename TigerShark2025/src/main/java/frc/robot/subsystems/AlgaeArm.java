// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.motors.function.AngularControlMotor;
import org.frc5010.common.motors.hardware.GenericTalonFXMotor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard;

/** Add your docs here. */
public class AlgaeArm extends GenericSubsystem {
    protected AngularControlMotor motor;
    protected Angle safeDistance = Degrees.of(10);
    protected PIDControlType controlType = PIDControlType.NONE;
    protected State state = State.POSITION;
    
    public static enum State {
        POSITION,
        DEPLOY,
        RETRACT
    }

    public static enum Position {
        UP(Degrees.of(90)),
        DOWN(Degrees.of(-45)),
        L2(Degrees.of(30)),
        L3(Degrees.of(45)),;

        private final Angle position;

        private Position(Angle position) {
            this.position = position;
        }

        public Angle position() {
            return position;
        }
    }

    public static class Config {
        public int canId = 14;
        public int gearing = 6;
        public double mass = 0.25;
        public double length = 18.0;
        public double maxAngle = -45.0;
        public double startAngle = 90;
        public double minAngle = 90;
        public double kG = -0.147800;
        public double conversion = 360;
        public double xPos = 8.0;
        public double yPos = 2.875;
        public double zPos = 16.25;
        public GenericPID pid = new GenericPID(2, 0.0, 0.0);
        public MotorFeedFwdConstants feedFwd = new MotorFeedFwdConstants(0.02728, 0.11622, 0.00094272);
        public double statorCurrentLimit = 160.0;
        public double supplyCurrentLimit = 50.0;
        public double maxVelocity = 83.333;
        public double maxAcceleration = 16.66;
        public boolean invert = true;
    }
    private Config config = new Config();

    public AlgaeArm(Mechanism2d mechanismSimulation, Config config) {
        super(mechanismSimulation);
        if (config != null) this.config = config;
       
        motor = new AngularControlMotor(MotorFactory.TalonFX(config.canId, Motor.KrakenX60), "Algae Arm", displayValues);
        motor.setupSimulatedMotor(config.gearing, Pounds.of(config.mass).in(Kilograms), Inches.of(config.length), Degrees.of(config.maxAngle), Degrees.of(config.minAngle),
                true, config.kG, Degrees.of(config.startAngle), false, config.conversion);
        motor.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(config.xPos).in(Meters), Inches.of(config.yPos).in(Meters), Inches.of(config.zPos).in(Meters)),
                new Rotation3d()));

        motor.setMotorBrake(true);
        motor.setValues(config.pid);
        motor.setMotorFeedFwd(config.feedFwd);
        motor.setCurrentLimit(Amps.of(config.statorCurrentLimit));
        ((GenericTalonFXMotor) motor.getMotorController()).setSupplyCurrent(Amps.of(config.supplyCurrentLimit));
        motor.setControlType(controlType);
        motor.setProfiledMaxVelocity(config.maxVelocity);
        motor.setProfiledMaxAcceleration(config.maxAcceleration);
        motor.invert(config.invert);

        // Ensure this angle works
        motor.getMotorEncoder().setPosition(config.startAngle);
    }

    public static Trigger algaeSelected = new Trigger(() -> ReefscapeButtonBoard.getCurrentAlignment() == ReefscapeButtonBoard.ScoringAlignment.ALGAE);

    public Command getDeployCommand() {
        return driveToAngleCommand(-10.0);
    }

    public void armSpeed(double speed) {
        if (motor.getControlType() != PIDControlType.NONE && speed != 0) {
            motor.setControlType(PIDControlType.NONE);
        }
        if (PIDControlType.NONE == motor.getControlType()) {
            // elevator.setReference(speed);
            motor.set(speed + motor.getFeedForward(0).in(Volts) / RobotController.getBatteryVoltage());
        }
    }

    public Command getSysIdCommand() {
        return motor.getSysIdCommand(this);
    }

    public void setArmPosition(Position position) {
        if (motor.getControlType() != controlType) {
            motor.setControlType(controlType);
        }
        if (controlType == motor.getControlType()) {
            motor.setReference(position.position().in(Degrees));
        }
    }

    public Command getInitialCommand(DoubleSupplier inputSpeeDoubleSupplier){
        return Commands.run(()->{
            double armPosition = 90 - inputSpeeDoubleSupplier.getAsDouble() * 120;
            driveToAngle(armPosition);
            //motor.setReference(armPosition/60);
        }, this);
    }

    public Command driveToAngleCommand(Double position) {
        return Commands.run(() -> {
            driveToAngle(position);
        }, this);
    }

    public void driveToAngle(double position){
        double difference = position - motor.getPivotPosition();
            double sign = Math.signum(difference);
            double effort = 0.05;
            if (Math.abs(difference) < safeDistance.in(Degrees)) {
                effort *= Math.max(Math.abs(difference) / safeDistance.in(Degrees), 0.05);

                if (Math.abs(difference) < 5) {
                    effort = 0;
                }
            }
            effort *= sign;
            motor.set(effort);
    }

    public Trigger isAtTarget() {
        return new Trigger(() -> motor.isAtTarget());
    }

    @Override
    public void periodic() {
        motor.draw();
    }

    @Override
    public void simulationPeriodic() {
        motor.simulationUpdate();
    }
}
