package frc.robot;

import java.util.function.Consumer;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.auto.RelayPIDAutoTuner;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNav;
import org.frc5010.common.utils.AllianceFlip;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_routines.Right4Coral;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class Elphaba extends GenericRobot {
    GenericDrivetrain drivetrain;
    ElevatorSystem elevatorSystem;
    ShooterSystem shooter;
    AlgaeArm algaeArm;
    ReefscapeButtonBoard reefscapeButtonBoard;

    public Elphaba(String directory) {
        super(directory);
        AllianceFlip.configure(FieldConstants.fieldDimensions);

        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        elevatorSystem = new ElevatorSystem(mechVisual);
        shooter = new ShooterSystem(mechVisual);
        algaeArm = new AlgaeArm(mechVisual);

        reefscapeButtonBoard = new ReefscapeButtonBoard(2);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        if (DriverStation.isTest()) {
            driver.createAButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdDriveMotorCommand());
            driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdAngleMotorCommand());
            operator.createYButton().whileTrue(elevatorSystem.elevatorSysIdCommand());

            QuestNav calibrationQuest = new QuestNav(new Transform3d());
            driver.createXButton().whileTrue(calibrationQuest.determineOffsetToRobotCenter(drivetrain));

            driver.createYButton().whileTrue(shooter.getSysIdCommand());

            operator.createAButton().whileTrue(
                    new RelayPIDAutoTuner(
                            (Consumer<Double>) ((Double value) -> ((YAGSLSwerveDrivetrain) drivetrain)
                                    .driveFieldOriented(new ChassisSpeeds(value, 0, 0))),
                            () -> ((YAGSLSwerveDrivetrain) drivetrain).getPose().getTranslation().getX(),
                            3.5,
                            drivetrain));

            operator.createBButton().whileTrue(
                    new RelayPIDAutoTuner(
                            (Consumer<Double>) ((Double value) -> ((YAGSLSwerveDrivetrain) drivetrain)
                                    .drive(new ChassisSpeeds(0, 0, value))),
                            () -> ((YAGSLSwerveDrivetrain) drivetrain).getPose().getRotation().getRadians(),
                            3.14 * 5,
                            drivetrain));

            return;
        }
        reefscapeButtonBoard.configureOperatorButtonBindings(operator);
        driver.createXButton().whileTrue( // Test drive to J Reef Location
                Commands.deferredProxy(((YAGSLSwerveDrivetrain) drivetrain)
                        .driveToPosePrecise(() -> AllianceFlip.apply(ReefscapeButtonBoard.getScoringPose()))));

        driver.createYButton().whileTrue( // Test drive to Top Station Position 1
                Commands.deferredProxy(((YAGSLSwerveDrivetrain) drivetrain)
                        .driveToPosePrecise(() -> AllianceFlip.apply(ReefscapeButtonBoard.getStationPose()))));

        driver.createLeftBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .profiledBangBangCmd(
                        elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.getScoringLevel()))));

        driver.LEFT_BUMPER.and(AlgaeArm.algaeSelected).and(ReefscapeButtonBoard.algaeLevelIsSelected)
            .whileTrue(algaeArm.getDeployCommand());

        driver.createBButton().whileTrue(Commands.run(() -> algaeArm.armSpeed(1)));

        driver.createRightBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .profiledBangBangCmd(
                        elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.ScoringLevel.INTAKE))));

        driver.createRightPovButton().onTrue(elevatorSystem.zeroElevator());


    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
        reefscapeButtonBoard.configureOperatorButtonBindings(operator);

        shooter.setDefaultCommand(shooter.runMotors(() -> operator.getLeftTrigger()));

        elevatorSystem.setDefaultCommand(elevatorSystem.basicSuppliersMovement(operator::getLeftYAxis));
        algaeArm.setDefaultCommand(algaeArm.getInitialCommand(operator::getRightTrigger));

    }

    @Override
    public void setupTestDefaultCommmands(Controller driver, Controller operator) {
        drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
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
