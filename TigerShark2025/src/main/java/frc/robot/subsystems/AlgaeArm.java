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

    public AlgaeArm(Mechanism2d mechanismSimulation) {
        super(mechanismSimulation);

        motor = new AngularControlMotor(MotorFactory.TalonFX(14, Motor.KrakenX60), "Algae Arm", displayValues);
        motor.setupSimulatedMotor(6, Pounds.of(0.25).in(Kilograms), Inches.of(18), Degrees.of(-45), Degrees.of(90),
                true, 0.147800, Degrees.of(90), false, 360);
        motor.setVisualizer(mechanismSimulation, new Pose3d(
                new Translation3d(Inches.of(8).in(Meters), Inches.of(2.875).in(Meters), Inches.of(16.25).in(Meters)),
                new Rotation3d()));
        motor.setValues(new GenericPID(0.1, 0.0, 0.0));
        motor.setMotorFeedFwd(new MotorFeedFwdConstants(0.12, 0.12, 0));
        motor.setCurrentLimit(Amps.of(80));
        motor.setControlType(PIDControlType.NONE);
        motor.invert(true);

        // Ensure this angle works
        motor.getMotorEncoder().setPosition(90);
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
