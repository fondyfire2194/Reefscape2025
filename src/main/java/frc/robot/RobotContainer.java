// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.Factories.CommandFactory.CoralRPMSetpoints;
import frc.robot.Factories.CommandFactory.ElevatorSetpoints;
import frc.robot.Factories.CommandFactory.Setpoint;
import frc.robot.commands.Arm.JogArm;
import frc.robot.commands.Arm.PositionHoldArm;
import frc.robot.commands.Elevator.JogElevator;
import frc.robot.commands.Elevator.PositionHoldElevator;
import frc.robot.commands.auto.AutoToTag;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.teleopAutos.GetNearestCoralStationPose;
import frc.robot.commands.teleopAutos.GetNearestReefZonePose;
import frc.robot.commands.teleopAutos.PIDDriveToPose;
import frc.robot.commands.teleopAutos.TeleopToTagV2;
import frc.robot.commands.teleopAutos.TurnToReef;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorArmSim;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LedStrip;
import monologue.Logged;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {

        ArmSubsystem arm = new ArmSubsystem();

        ElevatorSubsystem elevator = new ElevatorSubsystem();

        GamepieceSubsystem gamepieces = new GamepieceSubsystem();

        ElevatorArmSim elasim;

        LedStrip ls = new LedStrip();

        SendableChooser<Command> autoChooser;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController coDriverXbox = new CommandXboxController(1);

        // The robot's subsystems and commands are defined here...
        final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve")); // "swerve"));

        CommandFactory cf = new CommandFactory(drivebase, elevator, arm, gamepieces, ls);

        Trigger reefZoneChange = new Trigger(() -> drivebase.reefZone != drivebase.reefZoneLast);

        Trigger coralAtIntake = new Trigger(() -> gamepieces.coralAtIntake());

        Trigger stickyFaulTrigger = new Trigger(
                        () -> gamepieces.getStickyFault() || arm.getStickyFault() || elevator.getStickyFault());

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the rotational velocity
        // buttons are quick rotation positions to different ways to face
        // WARNING: default buttons are on the same buttons as the ones defined in
        // configureBindings
        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                        OperatorConstants.LEFT_Y_DEADBAND),
                        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                        OperatorConstants.DEADBAND),
                        () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                        OperatorConstants.RIGHT_X_DEADBAND),
                        driverXbox.getHID()::getYButtonPressed,
                        driverXbox.getHID()::getAButtonPressed,
                        driverXbox.getHID()::getXButtonPressed,
                        driverXbox.getHID()::getBButtonPressed);

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) * Math.PI)
                                                        *
                                                        (Math.PI * 2))
                        .headingWhile(true);

        Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

        Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

        public final LimelightVision m_llv = new LimelightVision();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                NamedCommands.registerCommand("Elevator Arm To Coral Station",
                                cf.setSetpointCommand(Setpoint.kCoralStation));

                NamedCommands.registerCommand("Elevator Arm To Coral L4",
                                cf.setSetpointCommand(Setpoint.kLevel4));

                NamedCommands.registerCommand("Deliver Coral L123", gamepieces.deliverCoralCommandL123());

                NamedCommands.registerCommand("Deliver Coral L4", gamepieces.deliverCoralCommandL4());

                NamedCommands.registerCommand("Intake Coral", gamepieces.intakeCoralToSwitchCommand());

                NamedCommands.registerCommand("Gamepices Stop", gamepieces.stopMotorCommand());

                NamedCommands.registerCommand("Intake Algae", gamepieces.intakeAlgaeCommand());

                NamedCommands.registerCommand("Deliver Algae", gamepieces.deliverAlgaeCommand());

                NamedCommands.registerCommand("Intake Algae L2",
                                cf.setSetpointCommand(Setpoint.KAlgaePickUpL2));

                NamedCommands.registerCommand("Intake Algae L3",
                                cf.setSetpointCommand(Setpoint.kAlgaePickUpL3));

                NamedCommands.registerCommand("Deliver Processor",
                                cf.setSetpointCommand(Setpoint.kProcessorDeliver));

                NamedCommands.registerCommand("CompLL",
                                new AutoToTag(drivebase, m_llv));

                if (RobotBase.isSimulation())
                        elasim = new ElevatorArmSim(elevator, arm);

                // if (RobotBase.isSimulation())
                elevator.setDefaultCommand(new PositionHoldElevator(elevator, arm));
                // if (RobotBase.isSimulation())
                arm.setDefaultCommand(new PositionHoldArm(arm));

                // Configure the trigger bindings
                configureBindings();
                // reefZoneChange.onTrue(rumble(driverXbox, RumbleType.kLeftRumble, 1))
                // .onTrue(new InstantCommand(() -> drivebase.reefZoneLast =
                // drivebase.reefZone));

                coralAtIntake.onTrue(rumble(driverXbox, RumbleType.kRightRumble, 1));

                stickyFaulTrigger.onTrue(rumble(coDriverXbox, RumbleType.kBothRumble, 1));

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                // (Condition) ? Return-On-True : Return-on-False
                // drivebase.setDefaultCommand(
                // !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
                // driveFieldOrientedDirectAngleSim);
                drivebase.setDefaultCommand(
                                Commands.parallel(
                                                new GetNearestCoralStationPose(drivebase),
                                                new GetNearestReefZonePose(drivebase, ls),
                                                drivebase.driveCommand(
                                                                () -> -MathUtil.applyDeadband(
                                                                                driverXbox.getLeftY()
                                                                                                * getAllianceFactor(),
                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                () -> -MathUtil.applyDeadband(
                                                                                driverXbox.getLeftX()
                                                                                                * getAllianceFactor(),
                                                                                OperatorConstants.DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),

                                                                                OperatorConstants.RIGHT_X_DEADBAND))));

                if (DriverStation.isTeleop() || DriverStation.isTest()) {

                        driverXbox.b().whileTrue(gamepieces.deliverAlgaeCommand());

                        driverXbox.y().onTrue(gamepieces.intakeAlgaeCommand());

                        driverXbox.a().whileTrue(gamepieces.deliverCoralCommandL123());

                        driverXbox.x().onTrue(gamepieces.intakeCoralToSwitchCommand());

                        driverXbox.back().onTrue(drivebase.centerModulesCommand());
                        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

                        driverXbox.leftBumper().whileTrue(
                                        Commands.defer(() -> new PIDDriveToPose(drivebase,
                                                        drivebase.getFinalReefTargetPose()),
                                                        Set.of(drivebase)));

                        driverXbox.rightBumper().whileTrue(
                                        Commands.defer(() -> new TeleopToTagV2(drivebase, m_llv),
                                                        Set.of(drivebase)));

                        driverXbox.rightBumper().whileTrue(
                                        Commands.defer(() -> new TeleopToTagV2(drivebase, m_llv),
                                                        Set.of(drivebase)));

                        // driverXbox.leftTrigger().whileTrue(
                        // Commands.defer(() -> Commands.sequence(drivebase.setSide(Side.LEFT),
                        // new TeleopToTagV2(drivebase, m_llv, coDriverXbox)),
                        // Set.of(drivebase)));

                        // driverXbox.rightTrigger().whileTrue(
                        // Commands.defer(() -> Commands.sequence(drivebase.setSide(Side.RIGHT),
                        // new TeleopToTagV2(drivebase, m_llv, coDriverXbox)),
                        // Set.of(drivebase)));

                        // driverXbox.rightTrigger().whileTrue(
                        //                 Commands.defer(() -> new TeleopToTagV2(drivebase, m_llv),
                        //                                 Set.of(drivebase)));
                        driverXbox.rightTrigger().whileTrue(
                                        Commands.defer(() -> drivebase.driveToPose(new Pose2d(1, 1, new Rotation2d())),
                                                        Set.of(drivebase)));

                                                        

                        driverXbox.povDown().whileTrue(new TurnToReef(drivebase));
                } /*
                   * Codriver controls
                   * 
                   * dpad sets left center right
                   * abxy set levels
                   * 
                   */
                if (DriverStation.isTeleop()) {

                        coDriverXbox.a().onTrue(
                                        Commands.parallel(
                                                        cf.setSetpointCommand(Setpoint.kLevel1),
                                                        gamepieces.setTargetRPM(CoralRPMSetpoints.kReefPlaceL123)));
                        coDriverXbox.x().onTrue(
                                        Commands.parallel(
                                                        cf.setSetpointCommand(Setpoint.kLevel2),
                                                        gamepieces.setTargetRPM(CoralRPMSetpoints.kCoralStation)));
                        coDriverXbox.b().onTrue(Commands.none());

                        coDriverXbox.y().onTrue(
                                        Commands.parallel(
                                                        cf.setSetpointCommand(Setpoint.kLevel4),
                                                        gamepieces.setTargetRPM(CoralRPMSetpoints.kReefPlaceL4)));

                        coDriverXbox.povUp().onTrue(arm.setGoalDegreesCommand(ArmSetpoints.kProcessorDeliver));

                        coDriverXbox.povRight().onTrue(
                                        Commands.sequence(
                                                        drivebase.setSide(Side.RIGHT),
                                                        m_llv.setPOIRight(),
                                                        Commands.runOnce(() -> ls.setViewTwoSolidColor(Side.RIGHT))));

                        coDriverXbox.povLeft().onTrue(
                                        Commands.sequence(
                                                        drivebase.setSide(Side.LEFT),
                                                        m_llv.setPOILeft(),
                                                        Commands.runOnce(() -> ls.setViewTwoSolidColor(Side.LEFT))));

                        coDriverXbox.povDown().onTrue(
                                        Commands.sequence(
                                                        drivebase.setSide(Side.CENTER),
                                                        m_llv.clearPOI(),
                                                        Commands.runOnce(() -> ls.setViewTwoSolidColor(Side.CENTER))));

                        coDriverXbox.start().onTrue(
                                        Commands.parallel(
                                                        elevator.clearStickyFaultsCommand(),
                                                        arm.clearStickyFaultsCommand(),

                                                        gamepieces.clearStickyFaultsCommand()));

                }

                if (DriverStation.isTest()) {
                        coDriverXbox.leftBumper().whileTrue(new JogArm(arm, coDriverXbox));// leftY

                        coDriverXbox.rightBumper().whileTrue(new JogElevator(elevator, coDriverXbox));// rightY

                        coDriverXbox.leftTrigger().whileTrue(
                                        gamepieces.jogMotorCommand(coDriverXbox.getLeftX()))
                                        .onFalse(gamepieces.stopMotorCommand());

                        coDriverXbox.rightTrigger().whileTrue(Commands.none());

                        coDriverXbox.y().onTrue(
                                        Commands.runOnce(() -> elevator
                                                        .setGoalInches(ElevatorSetpoints.kHome)));
                        coDriverXbox.a().onTrue(
                                        Commands.runOnce(() -> elevator.setGoalInches(ElevatorSetpoints.kLevel1 / 2)));
                        coDriverXbox.b().onTrue(
                                        Commands.runOnce(() -> elevator
                                                        .setGoalInches(ElevatorSetpoints.kLevel2 / 2)));
                        // coDriverXbox.y().onTrue(
                        // Commands.runOnce(
                        // () -> elevator.setGoalInches(ElevatorSetpoints.kCoralStation)));

                        coDriverXbox.povLeft().onTrue(
                                        Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kCoralStation)));

                        coDriverXbox.povUp().onTrue(
                                        Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kLevel1)));

                        coDriverXbox.povDown().onTrue(Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kLevel4)));

                        // coDriverXbox.povRight()
                        // .onTrue(Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kHome)));

                }

                if (Robot.isSimulation()) {
                        driverXbox.start()
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(7.25, 7.25,
                                                                        Rotation2d.fromDegrees(180)))));

                        coDriverXbox.start()
                                        .onTrue(Commands.runOnce(() -> drivebase
                                                        .resetOdometry(new Pose2d(1, 2,
                                                                        Rotation2d.fromDegrees(0)))));

                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();
        }

        public void setDriveMode() {
                configureBindings();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        public Command rumble(CommandXboxController controller, RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> controller.getHID().setRumble(type,
                                                                                timeout)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> controller.getHID().setRumble(type, 0.0)));

        }

        private double getAllianceFactor() {
                return drivebase.isBlueAlliance() ? 1 : -1;
        }
}
