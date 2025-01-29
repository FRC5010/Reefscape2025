package frc.robot;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.camera.QuestNav;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos2.Left2Coral;
import frc.robot.autos2.Right2Coral;
import frc.robot.autos2.Right4Coral;


public class TigerShark extends GenericRobot {
    GenericDrivetrain drivetrain;

    public TigerShark(String directory) {
        super(directory);
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        driver.createAButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdDriveMotorCommand());
        driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain) drivetrain).sysIdAngleMotorCommand());

        // Command offsetCommand = (new QuestNav(new Transform3d())).determineOffsetToRobotCenter(drivetrain);
        // driver.createXButton().whileTrue(offsetCommand);
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
        addAutoToChooser("Left 2 Coral", new Left2Coral());
        addAutoToChooser("Right 2 Coral", new Right2Coral());
        addAutoToChooser("Right 4 Coral", new Right4Coral());
        // addAutoToChooser("Auto New", new ExampleAuto());
    }
}
