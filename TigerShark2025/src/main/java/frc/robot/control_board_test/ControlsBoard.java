package frc.robot.control_board_test;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;



public class ControlsBoard extends GenericRobot {
    TestShooter testShooter;
    PIDController5010 pidController;
    TestBoard testBoard;


    public ControlsBoard(String directory) {
        super(directory);
        testShooter = new TestShooter(mechVisual, new TestShooter.Config());


        testBoard = new TestBoard();
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {



    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
       testShooter.setDefaultCommand(testShooter.runMotors(driver::getLeftYAxis));
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
