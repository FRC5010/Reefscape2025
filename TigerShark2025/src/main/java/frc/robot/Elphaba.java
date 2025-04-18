package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.commands.calibration.WheelRadiusCharacterization;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.DrivePoseEstimator.State;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNav;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.subsystems.NewLEDSubsystem;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto_routines.AutoChoosers;
import frc.robot.auto_routines.CustomAuto;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class Elphaba extends GenericRobot {
        GenericDrivetrain drivetrain;
        ElevatorSystem elevatorSystem;
        ShooterSystem shooter;
        GenericGyro gyro;
        ReefscapeButtonBoard reefscapeButtonBoard;
        AutoChoosers autoChoosers;
        DigitalInput brainZero, brainOne;
        DigitalInput centerLineZero;
        Pose2d centerLineResetPose;
        NewLEDSubsystem leds;

        boolean safetyOverride = false;

        private final Distance ROBOT_WIDTH = Inches.of(34.75);
        Pose2d startingPose1, startingPose2, startingPose3;
        RobotStates robotStates;

        public Elphaba(String directory) {
                super(directory);
                AllianceFlip.configure(FieldConstants.fieldDimensions);

                CameraServer.startAutomaticCapture();
                drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
                brainZero = new DigitalInput(0);
                brainOne = new DigitalInput(1);

                leds = new NewLEDSubsystem(4, 28, Inches.of(27.0));

                gyro = (GenericGyro) subsystems.get(ConfigConstants.GYRO);

                ElevatorSystem.Config eleConfig = new ElevatorSystem.Config();
                eleConfig.gearing = 72.0 / 12.0;
                elevatorSystem = new ElevatorSystem(mechVisual, eleConfig);
                shooter = new ShooterSystem(mechVisual, new ShooterSystem.Config());

                elevatorSystem.setUpAccelerationConstraints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // T0-DO:
                                                                                                          // Setup
                                                                                                          // correct
                                                                                                          // parameters
                elevatorSystem.setUpElevatorVelocityFunction(0.0, 0.0, 0.0);

                shooter.setLoadZoneSupplier(() -> elevatorSystem.atLoading());

                TargetingSystem.setupParameters((YAGSLSwerveDrivetrain) drivetrain, shooter, elevatorSystem, null);

                reefscapeButtonBoard = new ReefscapeButtonBoard(2, 3);


                elevatorSystem.setRobotParameters(Meters.of(0.0059182), Meters.of(0.0141478), Meters.of(0.56), 0.167775,
                                0.822008, 0.210722);
                autoChoosers = new AutoChoosers(shuffleTab);

                ((YAGSLSwerveDrivetrain) drivetrain).setAccelerationSuppliers(
                                () -> elevatorSystem.getMaxForwardAcceleration(),
                                () -> elevatorSystem.getMaxBackwardAcceleration(),
                                () -> elevatorSystem.getMaxLeftAcceleration(),
                                () -> elevatorSystem.getMaxRightAcceleration());

                ((YAGSLSwerveDrivetrain) drivetrain).setVelocitySuppliers(() -> elevatorSystem.getMaxForwardVelocity(),
                                () -> elevatorSystem.getMaxBackwardVelocity(),
                                () -> elevatorSystem.getMaxRightVelocity(),
                                () -> elevatorSystem.getMaxLeftVelocity());

                centerLineResetPose = new Pose2d(
                                FieldConstants.Reef.Side.GH.getCenterFace().getTranslation().getMeasureX()
                                                .plus(Inches.of(113).minus(ROBOT_WIDTH.div(2))),
                                Inches.of(146.5).minus(ROBOT_WIDTH.div(2)), Rotation2d.fromDegrees(180));
                resetPositionToCenterLine();

                // segmentedLED = new SegmentedLedSystem(0, 30, mechVisual);
                // LEDStripSegment providerSegment = segmentedLED.addLedSegment("Pose
                // Providers", 0, 10, Color.WHITE);

                // ((YAGSLSwerveDrivetrain)
                // drivetrain).getPoseEstimator().displayOnLEDSegment(providerSegment, 10);

                // Pose2d obstaclePosition = DriverStation.getAlliance().get() == Alliance.Blue
                // ? new Pose2d(4.48945, 4.0259, new Rotation2d())
                // : new Pose2d(13.065, 4.0259, new Rotation2d());
                // Pose2d[] blueVertices = new Pose2d[] { new Pose2d(3.325, 3.313, new
                // Rotation2d()),
                // new Pose2d(3.325, 4.698, new Rotation2d()), new Pose2d(4.490, 5.332, new
                // Rotation2d()),
                // new Pose2d(5.655, 4.689, new Rotation2d()), new Pose2d(5.655, 3.332, new
                // Rotation2d()),
                // new Pose2d(4.490, 2.700, new Rotation2d()) };
                // Pose2d[] redVertices = new Pose2d[] { new Pose2d(11.905, 3.313, new
                // Rotation2d()),
                // new Pose2d(11.905, 4.698, new Rotation2d()), new Pose2d(13.07, 5.332, new
                // Rotation2d()),
                // new Pose2d(14.235, 4.689, new Rotation2d()), new Pose2d(14.235, 3.332, new
                // Rotation2d()),
                // new Pose2d(13.07, 2.700, new Rotation2d()) };
                // Pose2d[] vertices = DriverStation.getAlliance().get() == Alliance.Blue ?
                // blueVertices : redVertices;
                // Distance obstacleRadius = Inches.of(37.621),
                // maximumObstacleDimensionDeviation = Inches.of(4.8755),
                // robotRadius = Inches.of(24.22), maximumRobotDimensionDeviation =
                // Inches.of(7.0934);

                // ((YAGSLSwerveDrivetrain) drivetrain).setUpCircularObstacle(obstaclePosition,
                // vertices,
                // obstacleRadius.in(Meters), robotRadius.in(Meters),
                // maximumRobotDimensionDeviation.in(Meters),
                // maximumObstacleDimensionDeviation.in(Meters), 100);
                startingPose1 = new Pose2d(
                                FieldConstants.innerStartingLineX.plus(Inches.of(1.0)).minus(Inches.of(17.875))
                                                .in(Meters),
                                FieldConstants.fieldDimensions.fieldWidth.minus(Inches.of(17.875)).in(Meters),
                                new Rotation2d(Degrees.of(180)));
                startingPose2 = new Pose2d(
                                FieldConstants.innerStartingLineX.plus(Inches.of(1.0)).minus(Inches.of(17.875))
                                                .in(Meters),
                                FieldConstants.fieldDimensions.fieldWidth.in(Meters) / 2,
                                new Rotation2d(Degrees.of(180)));
                startingPose3 = new Pose2d(
                                FieldConstants.innerStartingLineX.plus(Inches.of(1.0)).minus(Inches.of(17.875))
                                                .in(Meters),
                                Inches.of(17.875).in(Meters), new Rotation2d(Degrees.of(180)));

                        }

        public void resetPositionToStart() {
                ((YAGSLSwerveDrivetrain) drivetrain).resetPose(AllianceFlip.apply(new Pose2d(
                                new Translation2d(
                                                RobotModel.HALF_ROBOT_SIZE.plus(
                                                                FieldConstants.Reef.Side.GH.centerFace.getMeasureX()),
                                                FieldConstants.Reef.Side.GH.centerFace.getMeasureY()),
                                Rotation2d.fromDegrees(180))));
        }

        public void resetPositionToDiagonalStart() {
                ((YAGSLSwerveDrivetrain) drivetrain)
                                .resetPose(AllianceFlip
                                                .apply(new Pose2d(new Translation2d(Meters.of(7.585), Meters.of(6.160)),
                                                                Rotation2d.fromDegrees(-135))));
        }

        public void resetPositionToCenterLine() {
                ((YAGSLSwerveDrivetrain) drivetrain).resetPose(AllianceFlip.apply(centerLineResetPose));
        }

        public void resetPosition(Pose2d position) {
                ((YAGSLSwerveDrivetrain) drivetrain).resetPose(AllianceFlip.apply(position));
        }

        private void configureTestButtonBindings(Controller driver, Controller operator) {
                driver.createAButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdDriveMotorCommand());
                driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdAngleMotorCommand());
                driver.createXButton().whileTrue(new WheelRadiusCharacterization((YAGSLSwerveDrivetrain) drivetrain));
                operator.createYButton().whileTrue(elevatorSystem.elevatorSysIdCommand());

                QuestNav calibrationQuest = new QuestNav(new Transform3d(new Translation3d(),
                                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-90))));
                driver.createYButton().whileTrue(calibrationQuest.determineOffsetToRobotCenter(drivetrain));

                operator.createAButton()
                                .whileTrue(drivetrain.getPoseEstimator().getCalibrationCommand(drivetrain, 1));


                driver.createUpPovButton().whileTrue(Commands
                                .run(() -> ((YAGSLSwerveDrivetrain) drivetrain)
                                                .drive(new ChassisSpeeds(0.1, 0, 0)), drivetrain));

                operator.createYButton().onTrue(TargetingSystem.driveXMetersQuest(Meters.of(1.0)));
        }

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                drivetrain.configureButtonBindings(driver, operator);
                if (DriverStation.isTest()) {// TODO: Move into Generic Robot
                        configureTestButtonBindings(driver, operator);
                        return;
                }
                reefscapeButtonBoard.configureOperatorButtonBindings(operator);

                // Auto drive to reef
                driver.createBButton().whileTrue(
                                Commands.deferredProxy(() -> TargetingSystem.createCoralScoringSequence(
                                                ReefscapeButtonBoard.getScoringPose(),
                                                ReefscapeButtonBoard.getScoringLevel())));
                // Auto drive to station
                driver.createYButton().whileTrue(
                                Commands.deferredProxy(
                                                () -> TargetingSystem.createLoadingSequence(
                                                                ReefscapeButtonBoard.getStationPose())));

                // driver.createLeftBumper().whileTrue(Commands.run(() ->
                // elevatorSystem.setElevatorPosition(elevatorSystem.selectElevatorLevel(() ->
                // ReefscapeButtonBoard.getScoringLevel())), elevatorSystem));

                driver.createRightPovButton().onTrue(Commands.runOnce(() -> {
                        safetyOverride = !safetyOverride;
                }).andThen(Commands.startEnd(() -> driver.setRumble(0.5), () -> driver.setRumble(0.0)).withTimeout(0.5).onlyIf(() -> safetyOverride)));

                
                Trigger safetiesOverrided = new Trigger(() -> safetyOverride);
                Trigger elevatorOkToRun = (shooter.entrySensorIsBroken().negate()).or(safetiesOverrided);
                
               
                elevatorSystem.isLoadingTrigger().and(shooter.coralCapturedOrAligned().negate())
                        .whileTrue(shooter.intakeCoral());
                driver.createLeftBumper().and(elevatorOkToRun).whileTrue(Commands.deferredProxy(() -> elevatorSystem
                                .pidControlCommand(
                                                elevatorSystem.selectElevatorLevel(
                                                                () -> ReefscapeButtonBoard.getScoringLevel()))));

        


                driver.createRightBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                                .pidControlCommand(
                                                elevatorSystem.selectElevatorLevel(
                                                                () -> ReefscapeButtonBoard.ScoringLevel.INTAKE))
                                ).finallyDo(() -> elevatorSystem.elevatorPositionZeroSequence().schedule()
                                ));


                driver.createDownPovButton().whileTrue(elevatorSystem.elevatorPositionZeroSequence());


                reefscapeButtonBoard.getFireButton()
                                .whileTrue(Commands.deferredProxy(
                                                () -> shooter.getShootCommand(ReefscapeButtonBoard.getScoringLevel())));
                // shooter.coralHasEntered().whileTrue(shooter.intakeCoral());
                // shooter.isFullyCaptured()
                // .whileTrue(elevatorSystem
                // .pidControlCommand(ElevatorSystem.Position.CORAL_EXTENSION.position())
                // .until(() -> elevatorSystem.isAtLocation(
                // ElevatorSystem.Position.CORAL_EXTENSION.position())).andThen(shooter.alignCoral()));

                Trigger elevatorAtCoralExtendPosition = new Trigger(() -> elevatorSystem
                                .isAtLocationImproved(ElevatorSystem.Position.CORAL_EXTENSION.position()));
                reefscapeButtonBoard.getHomeButton()
                                .whileTrue(elevatorSystem
                                                .pidControlCommand(ElevatorSystem.Position.CORAL_EXTENSION.position())
                                                .until(() -> elevatorSystem.isAtLocation(
                                                                ElevatorSystem.Position.CORAL_EXTENSION.position()))
                                                .andThen(shooter.runMotors(() -> 0.5)));

                // operator.createUpPovButton().onTrue(Commands.runOnce(() ->
                // resetPositionToStart()).ignoringDisable(true));
                // operator.createRightPovButton().onTrue(Commands.runOnce(() ->
                // resetPosition(startingPose1)).ignoringDisable(true));
                // operator.createDownPovButton().onTrue(Commands.runOnce(() ->
                // resetPosition(startingPose2)).ignoringDisable(true));
                // operator.createLeftPovButton().onTrue(Commands.runOnce(() ->
                // resetPosition(startingPose3)).ignoringDisable(true));

 
                YAGSLSwerveDrivetrain yagsl = (YAGSLSwerveDrivetrain) drivetrain;
                Trigger disabled = new Trigger(() -> DriverStation.isDisabled());
                operator.createLeftPovButton().and(disabled).onTrue(
                                Commands.runOnce(() -> yagsl.getPoseEstimator().setState(State.DISABLED_FIELD))
                                                .ignoringDisable(true));
                operator.createRightPovButton().and(disabled)
                                .onTrue(Commands.runOnce(() -> yagsl.getPoseEstimator().setState(State.ALL))
                                                .ignoringDisable(true));

                driver.createBackButton().onTrue(Commands.runOnce(() -> resetPositionToStart()).ignoringDisable(true));
                new Trigger(brainZero::get).onTrue(Commands.runOnce(() -> resetPositionToStart()).ignoringDisable(true)
                                .andThen(Commands.runOnce(() -> leds.setPattern(leds.getRainbowPattern(1.0))))
                                .withTimeout(Seconds.of(3.0)));
                new Trigger(brainOne::get).onTrue(Commands.runOnce(() -> resetPositionToDiagonalStart())
                                .ignoringDisable(true)
                                .andThen(Commands.runOnce(() -> leds.setPattern(leds.getRainbowPattern(1.0))))
                                .withTimeout(Seconds.of(3.0)));

                Trigger driverFire = new Trigger(() -> driver.getRightTrigger() > 0.15);
                driverFire.whileTrue(Commands
                                .deferredProxy(() -> shooter.getShootCommand(ReefscapeButtonBoard.getScoringLevel())));

                // TODO: Fix logix
                QuestNav.isQuestOn().and(() -> DriverStation.isDisabled()).whileTrue(
                                Commands.run(() -> leds.setPattern(leds.getSolidPattern(Color.kGreen)))
                                                .ignoringDisable(true));
                QuestNav.isQuestOn().negate().and(() -> DriverStation.isDisabled()).whileTrue(
                                Commands.run(() -> leds.setPattern(leds.getSolidPattern(Color.kRed)))
                                                .ignoringDisable(true));
                // QuestNav.hasHardReset().onTrue(
                // Commands.runOnce(() ->
                // leds.setPattern(leds.getMaskedPattern(leds.getCurrentPattern(), 0.5, 50.0)),
                // leds));
        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                // JoystickToSwerve driveCmd =
                // (JoystickToSwerve)drivetrain.createDefaultCommand(driver);
                Command driveCmd = ((YAGSLSwerveDrivetrain) drivetrain).driveWithSetpointGeneratorOrientationConsidered(
                                () -> ((YAGSLSwerveDrivetrain) drivetrain).getFieldVelocitiesFromJoystick(
                                                driver::getLeftYAxis,
                                                driver::getLeftXAxis, driver::getRightXAxis));

                drivetrain.setDefaultCommand(driveCmd);

                shooter.setDefaultCommand(shooter.runMotors(() -> operator.getLeftTrigger()));

                elevatorSystem.setDefaultCommand(elevatorSystem.basicSuppliersMovement(operator::getLeftYAxis));
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
                return drivetrain.generateAutoCommand(autoCommand);//.alongWith(algaeArm.getInitialCommand(() -> 0));
        }

        @Override
        public void buildAutoCommands() {
                super.buildAutoCommands();
                addAutoToChooser("Custom Auto", new CustomAuto());

        }
}
