// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoBalancing;
import frc.robot.autos.rotateNinety;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
        public final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController operator = new CommandXboxController(1);

        private static final int translationAxis = XboxController.Axis.kLeftY.value;
        private static final int strafeAxis = XboxController.Axis.kLeftX.value;
        private static final int rotationAxis = XboxController.Axis.kRightX.value;

        private final Swerve s_Swerve = new Swerve();
        private final Arm s_Arm = new Arm();
        final Pneumatics s_Pneumatics = new Pneumatics();
        public final LEDs s_LEDs = new LEDs();

        private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
        private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                        s_Swerve::getPose,
                        s_Swerve::resetOdometry,
                        Constants.SwerveConstants.swerveKinematics, // SwerveDriveKinematics
                        new PIDConstants(Constants.AutonomousConstants.kPXController, 0, 0),
                        new PIDConstants(Constants.AutonomousConstants.kPThetaController,
                                        0,
                                        0),
                        s_Swerve::setModuleStates,
                        eventMap,
                        true,
                        s_Swerve);

        private static Map<String, Command> eventMap = new HashMap<>();
        {
                eventMap.put("wait1Second", new WaitCommand(1));
                eventMap.put("autoBalance", new AutoBalancing(s_Swerve));
                eventMap.put("armExtend", new ArmPIDCommand(s_Arm, 110));
                eventMap.put("armMid", new ArmPIDCommand(s_Arm, 35));
                eventMap.put("armIn", new ArmPIDCommand(s_Arm, 1));
                eventMap.put("armUp", new InstantCommand(() -> s_Pneumatics.armReverse()));
                eventMap.put("armDown", new InstantCommand(() -> s_Pneumatics.armForward()));
                eventMap.put("pinchForward", new InstantCommand(() -> s_Pneumatics.pinchForward()));
                eventMap.put("pinchReverse", new InstantCommand(() -> s_Pneumatics.pinchReverse()));
                eventMap.put("white", new InstantCommand(() -> s_LEDs.whiteLED()));
                eventMap.put("green", new InstantCommand(() -> s_LEDs.greenLED()));
                eventMap.put("blue", new InstantCommand(() -> s_LEDs.blueLED()));
        }

        private final PathPlannerTrajectory CubeMobilityBalance = PathPlanner.loadPath("CubeMobilityBalance",
                        1.15,
                        3);

        private final PathPlannerTrajectory Balance = PathPlanner.loadPath("Balance",
                        1.15,
                        3);

        private final PathPlannerTrajectory Cube = PathPlanner.loadPath("Cube",
                        1,
                        3);

        private final PathPlannerTrajectory CubeMobility = PathPlanner.loadPath("CubeMobility",
                        1,
                        3);

        private final PathPlannerTrajectory CubeBalance = PathPlanner.loadPath("CubeBalance",
                        1.15,
                        3);
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                setDefaultCommands();
                configureButtonBindings();
                configureSmartDashboard();
                configureAutonomousPaths();

                s_LEDs.ConfigureLEDs();
                s_Pneumatics.ConfigurePneumatics();
        }

        private void setDefaultCommands() {
                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> driver.getRawAxis(rotationAxis),
                                                () -> driver.povDown().getAsBoolean(),
                                                () -> driver.leftBumper().getAsBoolean(),
                                                () -> driver.rightBumper().getAsBoolean()));
        }

        private void configureAutonomousPaths() {

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         * <p>
         * This method binds the buttons to commands.
         */
        private void configureButtonBindings() {
                /* Default Controls */
                driver.y().onTrue(new ArmPIDCommand(s_Arm, 100));
                driver.b().onTrue(new ArmPIDCommand(s_Arm, 35));
                driver.a().onTrue(new ArmPIDCommand(s_Arm, 2));
                driver.x().onTrue(new InstantCommand(() -> s_Pneumatics.armToggle()));
                driver.rightBumper().onTrue(new InstantCommand(() -> s_Pneumatics.pinchToggle()));
                driver.leftTrigger().whileTrue(new rotateNinety(s_Swerve));

                /* Force Controls */
                driver.leftBumper().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
                driver.povLeft().onTrue(new InstantCommand(() -> s_Arm.resetEncoder()));
                driver.povUp().whileTrue(new ArmCommand(s_Arm, 1));
                driver.povDown().whileTrue(new ArmCommand(s_Arm, -1));

                driver.povLeft().onTrue(new InstantCommand(() -> s_Pneumatics.compressorEnable()));
                driver.povRight().onTrue(new InstantCommand(() -> s_Pneumatics.compressorDisable()));
        }

        private void configureSmartDashboard() {
                autoChooser.setDefaultOption("Cube Anywhere", Cube);
                autoChooser.addOption("Cube + Mobility", CubeMobility);
                autoChooser.addOption("Auto Balance", Balance);
                autoChooser.addOption("Cube + Auto Balance", CubeBalance);
                autoChooser.addOption("Cube + Mobility + Auto Balance", CubeMobilityBalance);

                SmartDashboard.putData(autoChooser);
        }

        /**
         * Ran once the robot is put in disabled
         */
        public void disabledInit() {
                s_Swerve.resetToAbsolute();
                s_LEDs.grayLED();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                // Executes the autonomous command chosen in smart dashboard
                s_Swerve.getField().getObject("Field").setTrajectory(autoChooser.getSelected());
                return autoBuilder.fullAuto(autoChooser.getSelected());
        }

        public Pose2d getPose() {
                return s_Swerve.getPose();
        }
}