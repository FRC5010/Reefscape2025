// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.sensors.Beambreak;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;


public class ShooterSystem extends GenericSubsystem {
  Beambreak alignmentBeambreak;
  Beambreak entryBeambreak;

  protected VelocityControlMotor shooterLeft;
  protected VelocityControlMotor shooterRight;

  /** Creates a new Shooter. */
  public ShooterSystem(Mechanism2d mechanismSimulation) {
    alignmentBeambreak = new Beambreak(0);
    entryBeambreak = new Beambreak(1);

    shooterLeft = new VelocityControlMotor(MotorFactory.Spark(11, Motor.Neo), "shooterLeft", displayValues);
    shooterRight = new VelocityControlMotor(MotorFactory.TalonFX(12, Motor.KrakenX60), "shooterRight",
            displayValues);
    shooterLeft.setupSimulatedMotor(1, 10);
    shooterRight.setupSimulatedMotor(1, 10);
    shooterLeft.setVisualizer(mechanismSimulation, new Pose3d(
            new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters), Inches.of(16.25).in(Meters)),
            new Rotation3d()));
    shooterRight.setVisualizer(mechanismSimulation,
            new Pose3d(new Translation3d(Inches.of(7.15).in(Meters), Inches.of(2.875).in(Meters),
                    Inches.of(6.25).in(Meters)), new Rotation3d()));

  }

  public void shooterLeftSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void shooterRightSpeed(double speed) {
      shooterRight.setReference(speed * shooterRight.getMaxRPM().in(RPM));
  }

  @Override
  public void periodic() {
      shooterLeft.draw();
      shooterRight.draw();
  }

  @Override
  public void simulationPeriodic() {
      shooterLeft.simulationUpdate();
      shooterRight.simulationUpdate();
  }
}
