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
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ReefscapeButtonBoard.ScoringLocation;
import frc.robot.auto_routines.AutoChoosers;
import frc.robot.auto_routines.CustomAuto;
import frc.robot.auto_routines.DriveOnlyCustom;
import frc.robot.auto_routines.LudicrousMode;
import frc.robot.auto_routines.OneCoralCustom;
import frc.robot.auto_routines.WheelCalibrationAuto;
import frc.robot.managers.TargetingSystem;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

public class TigerShark extends GenericRobot {
        GenericSwerveDrivetrain drivetrain;
        ElevatorSystem elevatorSystem;
        ShooterSystem shooter;
        AlgaeArm algaeArm;
        ClimbSubsystem climb;
        GenericGyro gyro;
        ReefscapeButtonBoard reefscapeButtonBoard;
        AutoChoosers autoChoosers;
        DigitalInput brainZero, brainOne;
        DigitalInput centerLineZero;
        Pose2d centerLineResetPose;
        NewLEDSubsystem leds;

        boolean safetyOverride = false;

        private final Distance ROBOT_WIDTH = Inches.of(34.75);
        RobotStates robotStates;

        public TigerShark(String directory) {
                super(directory);
                AllianceFlip.configure(FieldConstants.fieldDimensions);

                CameraServer.startAutomaticCapture();
                drivetrain = (GenericSwerveDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
                brainZero = new DigitalInput(0);
                brainOne = new DigitalInput(1);

                leds = new NewLEDSubsystem(1, 28, Inches.of(27.0));

                gyro = (GenericGyro) subsystems.get(ConfigConstants.GYRO);

                ElevatorSystem.Config eleConfig = new ElevatorSystem.Config();
                eleConfig.gearing = 72.0 / 12.0;
                elevatorSystem = new ElevatorSystem(mechVisual, eleConfig);
                shooter = new ShooterSystem(mechVisual, new ShooterSystem.Config());
                algaeArm = new AlgaeArm(mechVisual, new AlgaeArm.Config());

                elevatorSystem.setUpAccelerationConstraints(-2.5, 1.0, 6, -2.5, 1.0, 6.0, -2.5, 1.0, 6.0); // T0-DO:
                                                                                                           // Setup
                                                                                                           // correct
                                                                                                           // parameters
                elevatorSystem.setUpElevatorVelocityFunction(1.0, 0.9, 0.0);

                shooter.setLoadZoneSupplier(() -> elevatorSystem.atLoading());

                TargetingSystem.setupParameters(drivetrain, shooter, elevatorSystem, algaeArm);

                reefscapeButtonBoard = new ReefscapeButtonBoard(2, 3);

                climb = new ClimbSubsystem();

                elevatorSystem.setRobotParameters(Meters.of(0.0059182), Meters.of(0.0141478), Meters.of(0.56), 0.167775,
                                0.822008, 0.210722);
                autoChoosers = new AutoChoosers(shuffleTab);

                drivetrain.setAccelerationSuppliers(
                                () -> elevatorSystem.getMaxForwardAcceleration(),
                                () -> elevatorSystem.getMaxBackwardAcceleration(),
                                () -> elevatorSystem.getMaxLeftAcceleration(),
                                () -> elevatorSystem.getMaxRightAcceleration());

                drivetrain.setVelocitySuppliers(() -> elevatorSystem.getMaxForwardVelocity(),
                                () -> elevatorSystem.getMaxBackwardVelocity(),
                                () -> elevatorSystem.getMaxRightVelocity(),
                                () -> elevatorSystem.getMaxLeftVelocity());

                centerLineResetPose = new Pose2d(
                                FieldConstants.Reef.Side.GH.getCenterFace().getTranslation().getMeasureX()
                                                .plus(Inches.of(113).minus(ROBOT_WIDTH.div(2))),
                                Inches.of(146.5).minus(ROBOT_WIDTH.div(2)), Rotation2d.fromDegrees(180));
                resetPositionToCenterLine();

                robotStates = new RobotStates(shooter, algaeArm, elevatorSystem, climb,
                                () -> drivetrain.getPoseEstimator().getCurrentPose());
                shooter.setElevatorHeightSupplier(() -> elevatorSystem.getElevatorPosition());
        }

        public void resetPositionToStart() {
                drivetrain.resetPose(AllianceFlip.apply(new Pose2d(
                                new Translation2d(
                                                RobotModel.HALF_ROBOT_SIZE.plus(
                                                                FieldConstants.Reef.Side.GH.centerFace.getMeasureX()),
                                                FieldConstants.Reef.Side.GH.centerFace.getMeasureY()),
                                Rotation2d.fromDegrees(180))));
        }

        public void resetPositionToDiagonalStart() {
                drivetrain.resetPose(AllianceFlip
                                .apply(new Pose2d(new Translation2d(Meters.of(7.585), Meters.of(6.160)),
                                                Rotation2d.fromDegrees(-135))));
        }

        public void resetPositionToCenterLine() {
                drivetrain.resetPose(AllianceFlip.apply(centerLineResetPose));
        }

        public void resetPosition(Pose2d position) {
                drivetrain.resetPose(AllianceFlip.apply(position));
        }

        private void configureTestButtonBindings(Controller driver, Controller operator) {
                driver.createAButton().whileTrue(drivetrain.sysIdDriveMotorCommand());
                driver.createBButton().whileTrue(drivetrain.sysIdAngleMotorCommand());
                driver.createXButton().whileTrue(new WheelRadiusCharacterization(drivetrain));
                operator.createYButton().whileTrue(elevatorSystem.elevatorSysIdCommand());

                QuestNav calibrationQuest = new QuestNav(new Transform3d(new Translation3d(),
                                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-90))));
                driver.createYButton().whileTrue(calibrationQuest.determineOffsetToRobotCenter(drivetrain));

                operator.createAButton()
                                .whileTrue(drivetrain.getPoseEstimator().getCalibrationCommand(drivetrain, 1));

                driver.createUpPovButton().whileTrue(Commands
                                .run(() -> drivetrain
                                                .drive(new ChassisSpeeds(0.1, 0, 0)), drivetrain));
                // operator.createXButton().whileTrue(algaeArm.getSysIdCommand());

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
                // driver.createYButton().whileTrue(
                // Commands.deferredProxy(
                // () -> TargetingSystem.createLoadingSequence(
                // ReefscapeButtonBoard.getStationPose())));

                driver.createYButton().whileTrue(
                                Commands.deferredProxy(() -> TargetingSystem.createNewTeleopCoralScoringSequence(
                                                ReefscapeButtonBoard.getScoringPose(),
                                                ReefscapeButtonBoard.getScoringLevel())));

                // driver.createLeftBumper().whileTrue(Commands.run(() ->
                // elevatorSystem.setElevatorPosition(elevatorSystem.selectElevatorLevel(() ->
                // ReefscapeButtonBoard.getScoringLevel())), elevatorSystem));

                driver.createRightPovButton().onTrue(Commands.runOnce(() -> {
                        safetyOverride = !safetyOverride;
                }).andThen(Commands.startEnd(() -> driver.setRumble(0.5), () -> driver.setRumble(0.0)).withTimeout(0.5)
                                .onlyIf(() -> safetyOverride)));

                operator.createLeftPovButton().onTrue(Commands.runOnce(() -> {
                        climb.setOverride(!climb.getOverride());
                }).andThen(Commands.startEnd(() -> operator.setRumble(0.5), () -> operator.setRumble(0.0))
                                .withTimeout(0.5)
                                .onlyIf(climb::getOverride)));

                Trigger safetiesOverrided = new Trigger(() -> safetyOverride);
                Trigger elevatorOkToRun = (shooter.entrySensorIsBroken().negate()).or(safetiesOverrided);
                Trigger isAuto = new Trigger(RobotState::isAutonomous);
                Trigger itAintAuto = isAuto.negate();

                elevatorSystem.isLoadingTrigger().and(shooter.coralCapturedOrAligned().negate()).and(itAintAuto)
                                .whileTrue(shooter.intakeCoral());
                driver.createLeftBumper().and(elevatorOkToRun).whileTrue(Commands.deferredProxy(() -> elevatorSystem
                                .pidControlCommand(
                                                elevatorSystem.selectElevatorLevel(
                                                                () -> ReefscapeButtonBoard.getScoringLevel()))));

                driver.LEFT_BUMPER.and(AlgaeArm.algaeSelected).and(ReefscapeButtonBoard.algaeLevelIsSelected)
                                .whileTrue(algaeArm.getDeployCommand());

                driver.createAButton().whileTrue(Commands.run(() -> algaeArm.armSpeed(1)));

                operator.createAButton().onTrue(Commands
                                .runOnce(() -> ReefscapeButtonBoard.setScoringLocation(ScoringLocation.PID_TEST)));

                driver.createRightBumper().whileTrue(Commands.deferredProxy(() -> elevatorSystem
                                .pidControlCommand(
                                                elevatorSystem.selectElevatorLevel(
                                                                () -> ReefscapeButtonBoard.ScoringLevel.INTAKE)))
                                .finallyDo(() -> elevatorSystem.elevatorPositionZeroSequence()
                                                .andThen(elevatorSystem.holdElevatorDown().withTimeout(1.0))
                                                .schedule()));

                driver.createDownPovButton().whileTrue(elevatorSystem.elevatorPositionZeroSequence());

                reefscapeButtonBoard.getFireButton()
                                .whileTrue(Commands.deferredProxy(
                                                () -> shooter.getShootCommand(ReefscapeButtonBoard.getScoringLevel())));

                // TODO: Try out this code
                // itAintAuto.and(shooter.coralHasEntered()).and(elevatorSystem.isLoadingTrigger()).whileTrue(shooter.intakeCoral());
                // itAintAuto.and(shooter.isFullyCaptured())
                // .onTrue(elevatorSystem
                // .pidControlCommand(ElevatorSystem.Position.CORAL_EXTENSION.position())
                // .until(() -> elevatorSystem.isAtLocation(
                // ElevatorSystem.Position.CORAL_EXTENSION.position())));

                Trigger elevatorAtCoralExtendPosition = new Trigger(() -> elevatorSystem
                                .isAtLocationImproved(ElevatorSystem.Position.CORAL_EXTENSION.position()));
                reefscapeButtonBoard.getHomeButton()
                                .whileTrue(elevatorSystem
                                                .pidControlCommand(ElevatorSystem.Position.CORAL_EXTENSION.position())
                                                .until(() -> elevatorSystem.isAtLocation(
                                                                ElevatorSystem.Position.CORAL_EXTENSION.position()))
                                                .andThen(shooter.runMotors(() -> 0.5)));

                climb.setDefaultCommand(climb.runClimb(operator::getRightYAxis));

                Trigger disabled = new Trigger(() -> DriverStation.isDisabled());
                operator.createLeftPovButton().and(disabled).onTrue(
                                Commands.runOnce(() -> drivetrain.getPoseEstimator().setState(State.DISABLED_FIELD))
                                                .ignoringDisable(true));
                operator.createRightPovButton().and(disabled)
                                .onTrue(Commands.runOnce(() -> drivetrain.getPoseEstimator().setState(State.ALL))
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
                Command driveCmd = drivetrain.driveWithSetpointGeneratorOrientationConsidered(
                                () -> drivetrain.getFieldVelocitiesFromJoystick(
                                                driver::getLeftYAxis,
                                                driver::getLeftXAxis, driver::getRightXAxis));

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
                return drivetrain.generateAutoCommand(autoCommand).alongWith(algaeArm.getInitialCommand(() -> 0));
        }

        @Override
        public void buildAutoCommands() {
                super.buildAutoCommands();
                addAutoToChooser("Custom Auto", new CustomAuto());
                addAutoToChooser("No Delay Custom Auto", new LudicrousMode());
                addAutoToChooser("No Elevator Custom", new DriveOnlyCustom());
                addAutoToChooser("One Coral Custom", new OneCoralCustom());
                addAutoToChooser("Wheel Calibration Auto",
                                new WheelCalibrationAuto(drivetrain));
        }
}