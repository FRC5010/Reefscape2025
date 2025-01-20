// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.FollowerMotor;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.motors.function.VerticalPositionControlMotor;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

/** Add your docs here. */
public class ScoringSystem extends GenericSubsystem {
    protected VerticalPositionControlMotor elevator;
    protected FollowerMotor elevatorFollower;
    protected VelocityControlMotor shooterLeft;
    protected VelocityControlMotor shooterRight;
    public ScoringSystem(Mechanism2d mechanismSimulation){
        shooterLeft = new VelocityControlMotor(MotorFactory.ThriftyNeo(11), "shooterLeft", displayValues);
        shooterRight = new VelocityControlMotor(MotorFactory.ThriftyNeo(12), "shooterRight", displayValues);
        shooterLeft.setupSimulatedMotor(3, 0.1);
        shooterRight.setupSimulatedMotor(3, 0.1);
        shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1,0.1,0.1),new Rotation3d()));
        shooterRight.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1,-0.1,0.1),new Rotation3d()));
        elevator = new VerticalPositionControlMotor(MotorFactory.Neo(9), "elevator", displayValues);
        elevatorFollower = new FollowerMotor(MotorFactory.Neo(10), elevator, "elevatorFollower");
        elevatorFollower.setInverted(true);
        elevator.setupSimulatedMotor(86, Kilograms.of(10), Meters.of(0.01), Meters.of(0), Meters.of(2), Meters.of(0), Meters.of(0.2), 0.1, 0.1);
        elevator.setVisualizer(mechanismSimulation, new Pose3d(new Translation3d(0.1, 0, 0.5),new Rotation3d()));
    }
    public void shooterLeftSpeed(double speed){
        shooterLeft.setReference(speed * shooterLeft.getMaxRPM().in(RPM));
    
    }
    public void shooterRightSpeed(double speed){
        shooterRight.setReference(speed * shooterRight.getMaxRPM().in(RPM));
    }
    public void elevatorSpeed(double speed){
        elevator.set(speed);
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
    