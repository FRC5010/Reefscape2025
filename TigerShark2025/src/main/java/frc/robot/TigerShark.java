package frc.robot;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoRoutines.Right4Coral;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class TigerShark extends GenericRobot {
    GenericDrivetrain drivetrain;
    ElevatorSystem elevatorSystem;
    ShooterSystem shooterSystem;
    AlgaeArm algaeArm;
    ReefscapeButtonBoard reefscapeButtonBoard;

    public TigerShark(String directory) {
        super(directory);
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        elevatorSystem = new ElevatorSystem(mechVisual);
        shooterSystem = new ShooterSystem(mechVisual);
        algaeArm = new AlgaeArm(mechVisual);
        reefscapeButtonBoard = new ReefscapeButtonBoard(2);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        reefscapeButtonBoard.configureOperatorButtonBindings(operator);

    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
        shooterSystem.setDefaultCommand(Commands.run(() -> {
            shooterSystem.shooterLeftSpeed(operator.getLeftTrigger());
            shooterSystem.shooterRightSpeed(operator.getRightTrigger());
            elevatorSystem.elevatorSpeed(operator.getLeftYAxis());
        }, shooterSystem));

        algaeArm.setDefaultCommand(Commands.run(() -> {
//            algaeArm.armSpeed(operator.getLeftYAxis());
        }, algaeArm));
        
        // Test commands
        operator.createAButton().whileTrue(Commands.run(() -> shooterSystem.shooterRightSpeed(0.5), shooterSystem));
        // operator.createAButton().whileTrue(
        //         Commands.runOnce(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.BOTTOM),
        //                 scoringSystem).until(scoringSystem.isAtTarget()));
        operator.createXButton().whileTrue(
                Commands.runOnce(() -> elevatorSystem.setElevatorPosition(ElevatorSystem.Position.L2), elevatorSystem)
                        .until(elevatorSystem.isAtTarget()));
        operator.createYButton().whileTrue(
                Commands.runOnce(() -> elevatorSystem.setElevatorPosition(ElevatorSystem.Position.L3), elevatorSystem)
                        .until(elevatorSystem.isAtTarget()));
        // operator.createBButton().whileTrue(
        //         Commands.run(() -> scoringSystem.setElevatorPosition(ScoringSystem.Position.L4), scoringSystem)
        //                 .until(scoringSystem.isAtTarget()));
        operator.createBButton().whileTrue(
                Commands.run(() -> algaeArm.setArmPosition(AlgaeArm.Position.L3), algaeArm)
                        .until(algaeArm.isAtTarget()));
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
