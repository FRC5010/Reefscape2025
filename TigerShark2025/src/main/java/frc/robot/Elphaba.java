package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Meter;

import java.util.function.Consumer;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.RelayPIDAutoTuner;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNav;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.utils.AllianceFlip;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_routines.AutoChoosers;
import frc.robot.auto_routines.Coral2;
import frc.robot.auto_routines.CustomAuto;
import frc.robot.auto_routines.Right1Coral;
import frc.robot.auto_routines.Right4Coral;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class Elphaba extends GenericRobot {
    GenericDrivetrain drivetrain;
    ElevatorSystem elevatorSystem;
    ShooterSystem shooter;
    AlgaeArm algaeArm;
    GenericGyro gyro;
    ReefscapeButtonBoard reefscapeButtonBoard;
    AutoChoosers autoChoosers;

    public Elphaba(String directory) {
        super(directory);
        AllianceFlip.configure(FieldConstants.fieldDimensions);

        CameraServer.startAutomaticCapture();
        
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        

        gyro = (GenericGyro) subsystems.get(ConfigConstants.GYRO);

        elevatorSystem = new ElevatorSystem(mechVisual, new ElevatorSystem.Config());

        elevatorSystem.setCOGFunctionParameters(0.233035, 0.824063, 0.189682);

        shooter = new ShooterSystem(mechVisual, new ShooterSystem.Config());
        algaeArm = new AlgaeArm(mechVisual, new AlgaeArm.Config());

        elevatorSystem.setRobotParameters(Meters.zero(), Inches.of(1.178), Meters.of(0.56), 0.233035, 0.824063, 0.189682);

        TargetingSystem.setupParameters((YAGSLSwerveDrivetrain) drivetrain, shooter, elevatorSystem, algaeArm);

        reefscapeButtonBoard = new ReefscapeButtonBoard(2, 3);
        autoChoosers = new AutoChoosers(shuffleTab);

        ((YAGSLSwerveDrivetrain) drivetrain).setAccelerationSuppliers(() -> elevatorSystem.getMaxForwardAcceleration(), () -> elevatorSystem.getMaxBackwardAcceleration(), () -> elevatorSystem.getMaxLeftAcceleration(), () -> elevatorSystem.getMaxRightAcceleration());

        ((YAGSLSwerveDrivetrain) drivetrain).setVelocitySuppliers(() -> elevatorSystem.getMaxForwardVelocity(), () -> elevatorSystem.getMaxBackwardVelocity(), () -> elevatorSystem.getMaxRightVelocity(), () -> elevatorSystem.getMaxLeftVelocity());

        Pose2d obstaclePosition = DriverStation.getAlliance().get() == Alliance.Blue ? new Pose2d(4.48945, 4.0259, new Rotation2d()) : new Pose2d(13.065, 4.0259, new Rotation2d());
        Pose2d[] blueVertices = new Pose2d[] {new Pose2d(3.325, 3.313, new Rotation2d()), new Pose2d(3.325, 4.698, new Rotation2d()), new Pose2d(4.490, 5.332, new Rotation2d()), new Pose2d(5.655, 4.689, new Rotation2d()), new Pose2d(5.655, 3.332, new Rotation2d()), new Pose2d(4.490, 2.700, new Rotation2d())};
        Pose2d[] redVertices = new Pose2d[] {new Pose2d(11.905, 3.313, new Rotation2d()), new Pose2d(11.905, 4.698, new Rotation2d()), new Pose2d(13.07, 5.332, new Rotation2d()), new Pose2d(14.235, 4.689, new Rotation2d()), new Pose2d(14.235, 3.332, new Rotation2d()), new Pose2d(13.07, 2.700, new Rotation2d())};
        Pose2d[] vertices = DriverStation.getAlliance().get() == Alliance.Blue ? blueVertices : redVertices;
        Distance obstacleRadius = Inches.of(37.621), maximumObstacleDimensionDeviation = Inches.of(4.8755), robotRadius = Inches.of(24.22), maximumRobotDimensionDeviation = Inches.of(7.0934);


        ((YAGSLSwerveDrivetrain) drivetrain).setUpCircularObstacle(obstaclePosition, vertices, obstacleRadius.in(Meters), robotRadius.in(Meters), maximumRobotDimensionDeviation.in(Meters), maximumObstacleDimensionDeviation.in(Meters), 100);
    }

    public void resetPositionToStart() {
        ((YAGSLSwerveDrivetrain) drivetrain).resetPose(new Pose2d(new Translation2d(Inches.of(-17).plus(FieldConstants.Reef.Side.AB.centerFace.getMeasureX()), FieldConstants.Reef.Side.GH.centerFace.getMeasureY()),
              Rotation2d.fromDegrees(0)));
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        drivetrain.configureButtonBindings(driver, operator);
        if (DriverStation.isTest()) {
            driver.createAButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdDriveMotorCommand());
            driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdAngleMotorCommand());
            operator.createYButton().whileTrue(elevatorSystem.elevatorSysIdCommand());

            QuestNav calibrationQuest = new QuestNav(new Transform3d(new Translation3d(), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-94))));
            driver.createXButton().whileTrue(calibrationQuest.determineOffsetToRobotCenter(drivetrain));

            driver.createYButton().whileTrue(shooter.getSysIdCommand());
            operator.createXButton().whileTrue(algaeArm.getSysIdCommand());

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



        driver.createXButton().whileTrue(
        Commands.deferredProxy(() ->
        TargetingSystem.createCoralScoringSequence(AllianceFlip.apply(ReefscapeButtonBoard.getScoringPose()), ReefscapeButtonBoard.getScoringLevel())));

        driver.createYButton().whileTrue(
            Commands.deferredProxy( () ->
        TargetingSystem.createLoadingSequence(AllianceFlip.apply(ReefscapeButtonBoard.getStationPose())) 
        ));
        
        

        //driver.createLeftBumper().whileTrue(Commands.run(() -> elevatorSystem.setElevatorPosition(elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.getScoringLevel())), elevatorSystem));
        driver.createLeftBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .pidControlCommand(
                        elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.getScoringLevel()))));

        driver.LEFT_BUMPER.and(AlgaeArm.algaeSelected).and(ReefscapeButtonBoard.algaeLevelIsSelected)
            .whileTrue(algaeArm.getDeployCommand());

       driver.createBButton().whileTrue(Commands.run(() -> algaeArm.armSpeed(1)));

       driver.createRightBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                .pidControlCommand(
                        elevatorSystem.selectElevatorLevel(() -> ReefscapeButtonBoard.ScoringLevel.INTAKE))));

        driver.createRightPovButton().onTrue(elevatorSystem.zeroElevator());
        driver.createDownPovButton().whileTrue(elevatorSystem.elevatorPositionZeroSequence());
        driver.createUpPovButton().whileTrue(Commands.run(()->{
            shooter.shooterLeftSpeed(1.0);
            shooter.shooterRightSpeed(0.7);
        }, shooter));


        reefscapeButtonBoard.getFireButton().whileTrue(shooter.runMotors(() -> 0.5));

        driver.createAButton().onTrue(Commands.runOnce(() -> resetPositionToStart()));
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        // JoystickToSwerve driveCmd = (JoystickToSwerve)drivetrain.createDefaultCommand(driver);
        Command driveCmd = ((YAGSLSwerveDrivetrain) drivetrain).driveWithSetpointGeneratorFieldRelative(() -> ((YAGSLSwerveDrivetrain) drivetrain).getFieldVelocitiesFromJoystick(driver::getLeftYAxis, driver::getLeftXAxis, driver::getRightXAxis));

        drivetrain.setDefaultCommand(driveCmd);

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
        addAutoToChooser("Right 4 Coral", new Right4Coral());
        addAutoToChooser("Right 1 Coral", new Right1Coral(((YAGSLSwerveDrivetrain)drivetrain), shooter, elevatorSystem));
        addAutoToChooser("2 Piece Coral", new Coral2(((YAGSLSwerveDrivetrain)drivetrain), shooter, elevatorSystem));
        addAutoToChooser("Custom Auto", Commands.deferredProxy(() -> new CustomAuto()));
    }
}
