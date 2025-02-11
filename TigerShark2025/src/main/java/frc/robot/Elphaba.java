package frc.robot;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.utils.AllianceFlip;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_routines.Right4Coral;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class Elphaba extends GenericRobot {
    GenericDrivetrain drivetrain;
    ElevatorSystem elevatorSystem;
    ShooterSystem shooter;
    // AlgaeArm algaeArm;
    ReefscapeButtonBoard reefscapeButtonBoard;

    public Elphaba(String directory) {
        super(directory);
        AllianceFlip.configure(FieldConstants.fieldDimensions);

        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        elevatorSystem = new ElevatorSystem(mechVisual);
        shooter = new ShooterSystem(mechVisual);
        // algaeArm = new AlgaeArm(mechVisual);

        reefscapeButtonBoard = new ReefscapeButtonBoard(2);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        reefscapeButtonBoard.configureOperatorButtonBindings(operator);
        driver.createLeftBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .profiledBangBangCmd(elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.getScoringLevel()))));
        driver.createRightBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .profiledBangBangCmd(elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.ScoringLevel.INTAKE))));
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
        reefscapeButtonBoard.configureOperatorButtonBindings(operator);

        shooter.setDefaultCommand(shooter.runMotors(() -> operator.getLeftTrigger()));

        // operator.createAButton().whileTrue(
        // Commands.runOnce(() ->
        // scoringSystem.setElevatorPosition(ScoringSystem.Position.BOTTOM),
        // scoringSystem).until(scoringSystem.isAtTarget()));
        // operator.createXButton().whileTrue(
        // Commands.runOnce(() ->
        // elevatorSystem.setElevatorPosition(ElevatorSystem.Position.L2),
        // elevatorSystem)
        // .until(elevatorSystem.isAtTarget()));
        // operator.createYButton().whileTrue(
        // Commands.runOnce(() ->
        // elevatorSystem.setElevatorPosition(ElevatorSystem.Position.L3),
        // elevatorSystem)
        // .until(elevatorSystem.isAtTarget()));
        // operator.createBButton().whileTrue(
        // Commands.run(() ->
        // scoringSystem.setElevatorPosition(ScoringSystem.Position.L4), scoringSystem)
        // .until(scoringSystem.isAtTarget()));

        elevatorSystem.setDefaultCommand(elevatorSystem.basicSuppliersMovement(operator::getLeftYAxis));
        // algaeArm.setDefaultCommand(algaeArm.getInitialCommand(operator::getRightTrigger));

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
