// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import static edu.wpi.first.units.Units.Meters;

import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayValuesHelper;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
        kG = new DisplayDouble(0.0, K_G, visualName);
        setControlType(PIDControlType.POSITION);
    }


    @Override
    public Command getSysIdCommand(SubsystemBase subsystemBase) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSysIdCommand'");
    }
    
}
