package frc.robot;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoRoutines.Right4Coral;
import frc.robot.subsystems.ScoringSystem;

public class TigerShark extends GenericRobot {
    GenericDrivetrain drivetrain;
    ScoringSystem scoringSystem;
    ReefscapeButtonBoard reefscapeButtonBoard;

    public TigerShark(String directory) {
        super(directory);
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        scoringSystem = new ScoringSystem(mechVisual);
        reefscapeButtonBoard = new ReefscapeButtonBoard(2);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {

    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
        scoringSystem.setDefaultCommand(Commands.run(() -> {
            scoringSystem.shooterLeftSpeed(operator.getLeftTrigger());
            scoringSystem.shooterRightSpeed(operator.getRightTrigger());
            scoringSystem.elevatorSpeed(operator.getLeftYAxis());
        }, scoringSystem));
        // Test commands
        operator.createAButton().whileTrue(Commands.run(() -> scoringSystem.shooterRightSpeed(0.5), scoringSystem));
        // operator.createAButton().whileTrue(
        //         Commands.runOnce(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.BOTTOM),
        //                 scoringSystem).until(scoringSystem.isAtTarget()));
        operator.createXButton().whileTrue(
                Commands.runOnce(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.L2), scoringSystem)
                        .until(scoringSystem.isAtTarget()));
        operator.createYButton().whileTrue(
                Commands.runOnce(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.L3), scoringSystem)
                        .until(scoringSystem.isAtTarget()));
        operator.createBButton().whileTrue(
                Commands.run(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.L4), scoringSystem)
                        .until(scoringSystem.isAtTarget()));
    }

    @Override
    public void initAutoCommands() {
        drivetrain.setAutoBuilder();
    }

    @Override
    public Command generateAutoCommand(Command autoCommand) {
        return drivetrain.generateAutoCommand(autoCommand);
    }

    @Override
    public void buildAutoCommands() {
        super.buildAutoCommands();
           addAutoToChooser("Right 4 Coral", new Right4Coral().raceWith(new AutoErrorTracker()));
        // addAutoToChooser("Auto New", new ExampleAuto());
    }
}
