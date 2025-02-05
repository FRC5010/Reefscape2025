package org.frc5010.common.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import org.frc5010.common.subsystems.PhysicsSim.SimProfile;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Holds information about a simulated TalonFX.
 */
class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation
    private final TalonFXSimState _talonFXSim;
    private double drumRadius = Inches.of(1.1).in(Meters);
    private double gearing = 36.0;
    private final ElevatorSim _motorSim;

    /**
     * Creates a new simulation profile for a TalonFX device.
     * 
     * @param talonFX
     *                        The TalonFX device
     * @param rotorInertia
     *                        Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia) {
        this._talonFXSim = talonFX.getSimState();
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(gearbox, Pounds.of(15).in(Kilograms),
                        drumRadius, gearing),
                gearbox, 0.0, 3.0, true,
                0.0);
        
        //new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
    }
 
    /**
     * Runs the simulation profile.
     * 
     * This uses very rudimentary physics simulation and exists to allow users to
     * test features of our products in simulation using our examples out of the
     * box. Users may modify this to utilize more accurate physics simulation.
     */
    public void run() {
        /// DEVICE SPEED SIMULATION

        _motorSim.setInputVoltage(_talonFXSim.getMotorVoltage());

        _motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double position_rot = _motorSim.getPositionMeters() * gearing / (drumRadius * 2 * Math.PI);
        final double velocity_rps = _motorSim.getVelocityMetersPerSecond() * gearing / (drumRadius * 2 * Math.PI);

        _talonFXSim.setRawRotorPosition(position_rot);
        _talonFXSim.setRotorVelocity(velocity_rps);

        _talonFXSim.setSupplyVoltage(12 - _talonFXSim.getSupplyCurrent() * kMotorResistance);
    }
}