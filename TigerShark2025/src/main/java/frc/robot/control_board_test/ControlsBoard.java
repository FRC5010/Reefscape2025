package frc.robot.control_board_test;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.motors.MotorConstants;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class ControlsBoard extends GenericRobot {
    MotorController5010 kraken;
    PIDController5010 pidController;
    TestBoard testBoard;


    public ControlsBoard(String directory) {
        super(directory);
        kraken = MotorFactory.TalonFX(14, MotorConstants.Motor.KrakenX60);

        pidController = kraken.getPIDController5010();
        pidController.setControlType(PIDControlType.PROFILED_VELOCITY);
        pidController.setValues(new GenericPID(0.00014344, 0, 0));
        pidController.setMotorFeedFwd(new MotorFeedFwdConstants(0.02728, 0.11622, 0.00094272));
        pidController.setProfiledMaxVelocity(5000);
        pidController.setProfiledMaxAcceleration(1000);

        testBoard = new TestBoard();
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        driver.createAButton().whileTrue(
            SystemIdentification.getSysIdFullCommand(
                SystemIdentification.angleSysIdRoutine(kraken, kraken.getMotorEncoder(), "Vertical Motor", testBoard),
                4,
                2,
                3)
        );

        testBoard.setDefaultCommand(Commands.run(
            () -> {
                pidController.setReference(driver.getLeftYAxis()*10);
            }, testBoard));

    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
       
    }

    @Override
    public void initAutoCommands() {

    }

    

    @Override
    public Command generateAutoCommand(Command autoCommand) {
        return Commands.none();
    }

    @Override
    public void buildAutoCommands() {

    }
}
