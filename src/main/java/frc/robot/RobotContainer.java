// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.Factories.CommandFactory.ElevatorSetpoints;
import frc.robot.Factories.CommandFactory.Setpoint;
import frc.robot.commands.Arm.JogArm;
import frc.robot.commands.Arm.PositionHoldArmPID;
import frc.robot.commands.Climber.JogClimber;
import frc.robot.commands.Elevator.JogElevator;
import frc.robot.commands.Elevator.PositionHoldElevatorPID;
import frc.robot.commands.Gamepieces.IntakeCoralToSwitch;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.teleopAutos.GetNearestCoralStationPose;
import frc.robot.commands.teleopAutos.GetNearestReefZonePose;
import frc.robot.commands.teleopAutos.PIDDriveToPose;
import frc.robot.commands.teleopAutos.TurnToReef;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorArmSim;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.PreIntakeSubsystem;
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

        ClimberSubsystem climber = new ClimberSubsystem();

        GamepieceSubsystem gamepieces = new GamepieceSubsystem();

        PreIntakeSubsystem preIn = new PreIntakeSubsystem();

        LimelightVision llv = new LimelightVision();

        ElevatorArmSim elasim;

        LedStrip ls = new LedStrip();

        SendableChooser<Command> autoChooser;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController coDriverXbox = new CommandXboxController(1);

        // The robot's subsystems and commands are defined here...
        final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve")); // "swerve"));

        CommandFactory cf = new CommandFactory(drivebase, elevator, arm, gamepieces, llv, ls, driverXbox, coDriverXbox);

        Trigger reefZoneChange = new Trigger(() -> drivebase.reefZone != drivebase.reefZoneLast);

        // Trigger coralAtIntake = new Trigger(() -> gamepieces.coralAtIntake());

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

                NamedCommands.registerCommand("Elevator Arm To Travel",
                                cf.setSetpointCommand(Setpoint.kTravel));

                new EventTrigger("Check Tag Values").whileTrue(Commands.run(() -> llv.getTXOKDeliverCoral()));

                /*
                 * Runs after robot arrives at reef
                 * Determines from front camera if L4 deliver is possible, otherwise does L1
                 * deliver
                 * Ends after coral delivered and arm and elevator return home
                 * 
                 * /**
                 * Runs later in paths approching coral station
                 */

                new EventTrigger("Elevator Arm to Travel").onTrue(cf.setSetpointCommand(Setpoint.kTravel));

                NamedCommands.registerCommand("Deliver Coral L4", cf.deliverCoralL4());

                NamedCommands.registerCommand("DelayStartIntake",
                                Commands.sequence(
                                                Commands.waitSeconds(.5),
                                                new IntakeCoralToSwitch(gamepieces, arm, false)));

                NamedCommands.registerCommand("Intake Algae", gamepieces.intakeAlgaeCommand());

                NamedCommands.registerCommand("Deliver Algae", gamepieces.deliverAlgaeToProcessorCommand());

                NamedCommands.registerCommand("Intake Algae L2",
                                cf.setSetpointCommand(Setpoint.kAlgaePickUpL2));

                NamedCommands.registerCommand("Intake Algae L3",
                                cf.setSetpointCommand(Setpoint.kAlgaePickUpL3));

                NamedCommands.registerCommand("Deliver Processor",
                                cf.setSetpointCommand(Setpoint.kProcessorDeliver));

                if (RobotBase.isSimulation())
                        elasim = new ElevatorArmSim(elevator, arm);

                configureBindings();

                setDefaultCommands();

                setTriggerActions();

                DriverStation.silenceJoystickConnectionWarning(true);

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);

                CommandScheduler.getInstance()
                                .onCommandInitialize(
                                                command -> System.out
                                                                .println("Command initialized: " + command.getName()));
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                command -> System.out
                                                                .println("Command interrupted: " + command.getName()));
                CommandScheduler.getInstance()
                                .onCommandFinish(command -> System.out
                                                .println("Command finished: " + command.getName()));

        }

        private void setDefaultCommands() {

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

                elevator.setDefaultCommand(new PositionHoldElevatorPID(elevator, arm));
                // if (RobotBase.isSimulation())
                // arm.setDefaultCommand(new PositionHoldArm(arm));
                arm.setDefaultCommand(new PositionHoldArmPID(arm));

        }

        private void setTriggerActions() {

                // coralAtIntake.onTrue(Commands.parallel(
                // Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kTravel)),
                // rumble(driverXbox, RumbleType.kRightRumble, 1)));

                stickyFaulTrigger.onTrue(rumble(coDriverXbox, RumbleType.kBothRumble, 1));

                reefZoneChange.onTrue(rumble(driverXbox, RumbleType.kLeftRumble, .25))
                                .onTrue(Commands.runOnce(() -> drivebase.reefZoneLast = drivebase.reefZone));
        }

        private void configureBindings() {

                if (DriverStation.isTeleop()) {

                        driverXbox.a().onTrue(Commands.none());

                        driverXbox.b().onTrue(cf.deliverToBargeWithArmCommand().withName("Deliver Algae"));

                        driverXbox.x().onTrue(gamepieces.deliverAlgaeToBargeCommand().withName("Deliver Barge"));

                        driverXbox.y().onTrue(gamepieces.intakeAlgaeCommand().withName("Intake Algae"));

                        driverXbox.back().onTrue(drivebase.centerModulesCommand().withName("Center Modules"));

                        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro).withName("Zero Gyro"));

                        driverXbox.leftTrigger().onTrue(
                                        new IntakeCoralToSwitch(gamepieces, arm, false).withName("IntakeCoral"));

                        driverXbox.rightTrigger().onTrue(gamepieces.deliverCoralCommand().withName("Deliver Coral"));

                        driverXbox.leftBumper().whileTrue(
                                        Commands.defer(() -> Commands.sequence(
                                                        drivebase.setSide(Side.LEFT),
                                                        new PIDDriveToPose(drivebase, drivebase.reefTargetPose)),
                                                        Set.of(drivebase)))
                                        .onFalse(Commands.sequence(
                                                        drivebase.setSide(Side.CENTER),
                                                        m_llv.clearPOI()).withName("Left Reef PID"));

                        driverXbox.rightBumper().whileTrue(
                                        Commands.defer(() -> Commands.sequence(
                                                        drivebase.setSide(Side.RIGHT),
                                                        new PIDDriveToPose(drivebase, drivebase.reefTargetPose)),
                                                        Set.of(drivebase)))
                                        .onFalse(Commands.sequence(
                                                        drivebase.setSide(Side.CENTER),
                                                        m_llv.clearPOI()).withName("Right Reef PID"));

                        driverXbox.povDown().whileTrue(new TurnToReef(drivebase).withName("Turn To Reef"));
                } /*
                   * Codriver controls
                   * 
                   * dpad sets left center right
                   * abxy set levels
                   * 
                   */
                if (DriverStation.isTeleop()) {

                        coDriverXbox.a().onTrue(
                                        cf.setSetpointCommand(Setpoint.kLevel1).withName("Set L1"));

                        coDriverXbox.b().onTrue(
                                        cf.setSetpointCommand(Setpoint.kLevel2).withName("Set L2"));

                        coDriverXbox.x().onTrue(
                                        cf.setSetpointCommand(Setpoint.kLevel3).withName("Set L3"));

                        coDriverXbox.y().onTrue(
                                        cf.setSetpointCommand(Setpoint.kLevel4).withName("Set L4"));

                        coDriverXbox.povDown().onTrue(
                                        cf.setSetpointCommand(Setpoint.kProcessorDeliver).withName("Set Processor"));

                        coDriverXbox.povRight().onTrue(
                                        cf.setSetpointCommand(Setpoint.kCoralStation).withName("Set Coral Station"));

                        coDriverXbox.povLeft()
                                        .onTrue(cf.setSetpointCommand(Setpoint.kAlgaePickUpL2)
                                                        .withName("Set Algae Pickup L2"));

                        coDriverXbox.rightBumper()
                                        .onTrue(cf.homeElevatorAndArm().withName("Home Elevator Arm"));

                        coDriverXbox.leftBumper()
                                        .onTrue(cf.setSetpointCommand(Setpoint.kAlgaeDeliverBarge)
                                                        .withName("Set Algae Pickup L2"));

                        coDriverXbox.leftTrigger().whileTrue(
                                        Commands.defer(
                                                        () -> gamepieces.jogGamepieceMotorCommand(
                                                                        () -> coDriverXbox.getLeftY()),
                                                        Set.of(gamepieces)))
                                        .onFalse(gamepieces.stopGamepieceMotorsCommand());
                        // .whileTrue(drivebase.sysIdDriveMotorCommand());

                        coDriverXbox.rightTrigger()
                                        .onTrue(llv.setPOILeft());

                        coDriverXbox.start().onTrue(
                                        Commands.parallel(
                                                        elevator.clearStickyFaultsCommand(),
                                                        arm.clearStickyFaultsCommand(),
                                                        gamepieces.clearStickyFaultsCommand()));

                }

                if (DriverStation.isTest()) {

                        coDriverXbox.leftBumper().whileTrue(new JogArm(arm, coDriverXbox))
                                        .onFalse(Commands.runOnce(() -> arm.stop()));// leftY

                        coDriverXbox.rightBumper().whileTrue(new JogElevator(elevator, coDriverXbox))// rightY
                                        .onFalse(Commands.runOnce(() -> elevator.stop()));// leftY

                        coDriverXbox.rightTrigger().whileTrue(
                                        Commands.defer(
                                                        () -> gamepieces.jogCoralIntakeMotorsCommand(
                                                                        () -> coDriverXbox.getLeftX()),
                                                        Set.of(gamepieces)))
                                        .onFalse(gamepieces.stopGamepieceMotorsCommand());

                        coDriverXbox.leftTrigger().whileTrue(new JogClimber(climber, coDriverXbox))
                                        .onFalse(Commands.runOnce(() -> climber.stop()));

                        coDriverXbox.y().onTrue(
                                        elevator.setGoalInchesCommand(ElevatorSetpoints.kHome));

                        coDriverXbox.a().onTrue(elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel2));

                        coDriverXbox.b().onTrue(
                                        elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel4));

                        coDriverXbox.x().onTrue(
                                        elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel3));

                        coDriverXbox.povLeft().onTrue(
                                        Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kCoralStation)));

                        coDriverXbox.povUp().onTrue(
                                        Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kLevel1)));

                        coDriverXbox.povDown()
                                        .onTrue(Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kLevel4_2)));

                        coDriverXbox.povRight()
                                        .onTrue(Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kAlgaeIntake)));

                        coDriverXbox.leftStick().onTrue(gamepieces.deliverCoralCommand());

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
