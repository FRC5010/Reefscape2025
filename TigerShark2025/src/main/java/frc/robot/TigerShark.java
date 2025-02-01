package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Consumer;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.auto.AutoErrorTracker;
import org.frc5010.common.auto.RelayPIDAutoTuner;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNav;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoRoutines.Right4Coral;


public class TigerShark extends GenericRobot {
    GenericDrivetrain drivetrain;

    public TigerShark(String directory) {
        super(directory);
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
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

        driver.createXButton().whileTrue( // Test drive to J Reef Location
            ((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(new Pose2d(5.000, 5.250, new Rotation2d(Degrees.of(-120))))
            );

        driver.createYButton().whileTrue( // Test drive to Top Station Position 1
            ((YAGSLSwerveDrivetrain) drivetrain).driveToPosePrecise(new Pose2d(1.740, 7.245, new Rotation2d(Degrees.of(-55.000))))
            );
        
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
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
