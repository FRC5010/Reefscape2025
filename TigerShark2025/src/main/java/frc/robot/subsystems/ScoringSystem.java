// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.motors.function.VerticalPositionControlMotor;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

/** Add your docs here. */
public class ScoringSystem extends GenericSubsystem {
    protected VerticalPositionControlMotor elevator;
    protected FollowerMotor elevatorFollower;
    protected VelocityControlMotor shooterLeft;
    protected VelocityControlMotor shooterRight;

    public static enum Position {
        BOTTOM(Meters.of(0)),
        LOAD(Meters.of(0.05)),
        L1(Meters.of(0.1)),
        L2Algae(Meters.of(0.11)),
        L2(Meters.of(0.5)),
        L3Algae(Meters.of(1.1)),
        L3(Meters.of(1)),
        L4(Meters.of(1.5)),
        NET(Meters.of(2));

        private final Distance position;
        private Position(Distance position) {
            this.position = position;
        }
        public Distance position() {
            return position;
        }
    }
    public ScoringSystem(Mechanism2d mechanismSimulation){
        shooterLeft = new VelocityControlMotor(MotorFactory.TalonFX(11, Motor.KrakenX60), "shooterLeft", displayValues);
        shooterRight = new VelocityControlMotor(MotorFactory.TalonFX(12, Motor.KrakenX60), "shooterRight", displayValues);
        shooterLeft.setupSimulatedMotor(1, 0.1);
        shooterRight.setupSimulatedMotor(1, 0.1);
        shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1,0.1,0.1),new Rotation3d()));
        shooterRight.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1,-0.1,0.15),new Rotation3d()));

        elevator = new VerticalPositionControlMotor(MotorFactory.TalonFX(9, Motor.KrakenX60), "elevator", displayValues);
        elevatorFollower = new FollowerMotor(MotorFactory.TalonFX(10, Motor.KrakenX60), elevator, "elevatorFollower");
        elevatorFollower.setInverted(true);
        elevator.setupSimulatedMotor(6, Pounds.of(15), Inches.of(1.1), Meters.of(0), Meters.of(2), Meters.of(0), Meters.of(0.2), 0.1, 0.1);
        elevator.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1, 0, 0.5),new Rotation3d()));
        elevator.setMotorFeedFwd(new MotorFeedFwdConstants(0.02, 12 / Motor.KrakenX60.maxRpm.in(RPM), 0.0));
        elevator.setProfiledMaxVelocity(2.0);
        elevator.setProfiledMaxAcceleration(0.5);
        elevator.setValues(new GenericPID(0.1, 0, 0));
    }
    public void shooterLeftSpeed(double speed){
        shooterLeft.setReference(speed * shooterLeft.getMaxRPM().in(RPM));
    }
    public void shooterRightSpeed(double speed){
        shooterRight.setReference(speed * shooterRight.getMaxRPM().in(RPM));
    }
    public void elevatorSpeed(double speed) {
        if (elevator.getControlType() != PIDControlType.DUTY_CYCLE && speed != 0) {
            elevator.setControlType(PIDControlType.DUTY_CYCLE);
        }
        if (PIDControlType.DUTY_CYCLE == elevator.getControlType()) {
            elevator.set(speed);
        }
    }
    
    public void setElevatorPosition(Position position) {
        if (elevator.getControlType() != PIDControlType.PROFILED_POSITION) {
            elevator.setControlType(PIDControlType.PROFILED_POSITION);
        }
        if (PIDControlType.PROFILED_POSITION == elevator.getControlType()) {
            elevator.setReference(position.position().in(Meters));
        }
    }

    @Override
    public void periodic() {
        shooterLeft.draw();
        shooterRight.draw();
        elevator.draw();
    }

    @Override
    public void simulationPeriodic() {
        shooterLeft.simulationUpdate();
        shooterRight.simulationUpdate();
        elevator.simulationUpdate();
    }
}
    