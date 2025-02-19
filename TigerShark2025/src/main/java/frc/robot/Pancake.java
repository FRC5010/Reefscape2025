package frc.robot;

import java.util.function.Consumer;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.auto.RelayPIDAutoTuner;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.utils.AllianceFlip;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_routines.Right4Coral;


public class Pancake extends GenericRobot {
    GenericDrivetrain drivetrain;
    ReefscapeButtonBoard buttonBoard;

    public Pancake(String directory) {
        super(directory);
        AllianceFlip.configure(FieldConstants.fieldDimensions);

        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
        buttonBoard = new ReefscapeButtonBoard(2, 3);

        ((YAGSLSwerveDrivetrain) drivetrain).setAccelerationSuppliers(() -> 9.0, () -> 9.0, () -> 9.0, () -> 9.0);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        buttonBoard.configureOperatorButtonBindings(operator);

        // driver.createAButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdDriveMotorCommand());
        // driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdAngleMotorCommand());
        driver.createAButton().whileTrue(
            new RelayPIDAutoTuner(
                (Consumer<Double>)((Double value) -> 
                    ((YAGSLSwerveDrivetrain) drivetrain).driveFieldOriented(new ChassisSpeeds(value, 0, 0))
                ), 
                () -> ((YAGSLSwerveDrivetrain) drivetrain).getPose().getTranslation().getX(),
                3.5,
                drivetrain)
        );

        driver.createBButton().whileTrue(
            new RelayPIDAutoTuner(
                (Consumer<Double>)((Double value) -> 
                    ((YAGSLSwerveDrivetrain) drivetrain).drive(new ChassisSpeeds(0, 0, value))
                ), 
                () -> ((YAGSLSwerveDrivetrain) drivetrain).getPose().getRotation().getRadians(),
                3.14*5,
                drivetrain)
        );


        // Command offsetCommand = (new QuestNav(new Transform3d())).determineOffsetToRobotCenter(drivetrain);
        // driver.createXButton().whileTrue(offsetCommand);

        // driver.createXButton().whileTrue( // Test drive to J Reef Location
        //     ((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(new Pose2d(5.000, 5.250, new Rotation2d(Degrees.of(-120))))
        //     );

        driver.createXButton().whileTrue( // Test drive to J Reef Location
        Commands.deferredProxy(((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(() -> AllianceFlip.apply(ReefscapeButtonBoard.getScoringPose()))));

        // driver.createYButton().whileTrue( // Test drive to Top Station Position 1
        //     ((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(new Pose2d(1.740, 7.245, new Rotation2d(Degrees.of(-55.000))))
        //     );
        
        driver.createYButton().whileTrue( // Test drive to Top Station Position 1
        Commands.deferredProxy(((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(() -> AllianceFlip.apply(ReefscapeButtonBoard.getStationPose()))));
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        Command driveCmd = ((YAGSLSwerveDrivetrain) drivetrain).driveWithSetpointGeneratorFieldRelative(() -> ((YAGSLSwerveDrivetrain) drivetrain).getFieldVelocitiesFromJoystick(driver::getLeftXAxis, driver::getLeftYAxis, driver::getRightXAxis));

        drivetrain.setDefaultCommand(driveCmd);
        // drivetrain.setDefaultCommand(((YAGSLSwerveDrivetrain) drivetrain).driveWithSetpointGeneratorFieldRelative(() ->
        // {
        //     double xInput = driver.getAxisValue(XboxController.Axis.kLeftX.value)+0.001;
        //     double yInput = driver.getAxisValue(XboxController.Axis.kLeftY.value)+0.001;

        //     Translation2d inputTranslation = new Translation2d(xInput, yInput);
        //     double magnitude = inputTranslation.getNorm();
        //     Rotation2d angle = 0 != xInput || 0 != yInput ? inputTranslation.getAngle() : new Rotation2d();

        //     double curvedMagnitude = Math.pow(magnitude, 3);

        //     double turnSpeed = driver.getAxisValue(XboxController.Axis.kRightX.value);

        //     double xSpeed =
        //             curvedMagnitude
        //                 * angle.getCos()
        //                 * ((YAGSLSwerveDrivetrain)drivetrain).getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
        //         double ySpeed =
        //             curvedMagnitude
        //                 * angle.getSin()
        //                 * ((YAGSLSwerveDrivetrain)drivetrain).getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
        //         turnSpeed =
        //             turnSpeed * ((YAGSLSwerveDrivetrain)drivetrain).getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

        //     return new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        // }));
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
