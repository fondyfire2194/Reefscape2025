// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;
import java.util.jar.Attributes.Name;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Factories.CommandFactory;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.Factories.CommandFactory.Setpoint;
import frc.robot.commands.Arm.JogArm;
import frc.robot.commands.Arm.PositionHoldArmPID;
import frc.robot.commands.Elevator.JogElevator;
import frc.robot.commands.Elevator.PositionHoldElevatorPID;
import frc.robot.commands.Gamepieces.BackupOffSwitchAfterCoraIntake;
import frc.robot.commands.Gamepieces.DetectAlgaeWhileIntaking;
import frc.robot.commands.Gamepieces.IntakeCoralToSwitch;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopSwerve;
import frc.robot.commands.teleopAutos.GetNearestCoralStationPose;
import frc.robot.commands.teleopAutos.GetNearestReefZonePose;
import frc.robot.commands.teleopAutos.PIDDriveToPose;
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

        ElevatorArmSim elasim;

        LedStrip ls = new LedStrip();

        SendableChooser<Command> autoChooser;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        CommandXboxController coDriverXbox = new CommandXboxController(1);

        // The robot's subsystems and commands are defined here...
        final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve")); // "swerve"));

        LimelightVision llv = new LimelightVision();

        CommandFactory cf = new CommandFactory(drivebase, elevator, arm, gamepieces, llv, ls, driverXbox, coDriverXbox);

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

        public boolean correctAngle = false;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                if (RobotBase.isSimulation()) {
                        elasim = new ElevatorArmSim(elevator, arm);
                        DriverStation.silenceJoystickConnectionWarning(true);
                }

                setNamedCommands();

                setDefaultCommands();

                configureDriverBindings();

                buildAutoChooser();

                setTriggerActions();

                printCommandEvents();

        }

        private void setNamedCommands() {

                boolean pathsOnly = false;

                if (pathsOnly) {
                        NamedCommands.registerCommand("Deliver Coral L4", Commands.waitSeconds(1));

                        NamedCommands.registerCommand("DelayStartIntake",
                                        Commands.waitSeconds(1));
                }

                else {

                        NamedCommands.registerCommand("PoseCorrect",
                                        new PIDDriveToPose(drivebase,
                                                        new Pose2d(5.32, 2.89, Rotation2d.fromDegrees(120))));

                        NamedCommands.registerCommand("Deliver Coral L4", cf.deliverCoralL4());

                        NamedCommands.registerCommand("DelayStartIntake",
                                        Commands.sequence(
                                                        Commands.waitSeconds(.5),
                                                        new IntakeCoralToSwitch(gamepieces, arm, false))
                                                        .withName("DelayedIntakeCoral"));

                        NamedCommands.registerCommand("ArmToTravel",

                                        Commands.sequence(
                                                        Commands.runOnce(() -> SmartDashboard.putBoolean("AAAA", true)),

                                                        arm.setGoalDegreesCommand(ArmSetpoints.kLevel4_1)));
                        NamedCommands.registerCommand("SetPOIR", llv.setPOIRight());
                        NamedCommands.registerCommand("SetPOIL", llv.setPOILeft());
                        NamedCommands.registerCommand("ClearPOI", llv.clearPOI());

                        NamedCommands.registerCommand("Intake Algae L2",
                                        cf.pickupAlgaeL2()
                                                        .withName("IntakeAlgaeL2"));

                        NamedCommands.registerCommand("Intake Algae L3",
                                        cf.pickupAlgaeL3()
                                                        .withName("IntakeAlgaeL3"));

                        NamedCommands.registerCommand("ElevatorToProcessor",
                                        cf.setSetpointCommand(Setpoint.kProcessorDeliver));

                        NamedCommands.registerCommand("Deliver Processor",
                                        gamepieces.deliverAlgaeToProcessorCommand()
                                                        .withName("Deliver Processor"));

                        NamedCommands.registerCommand("Deliver Barge",
                                        cf.setSetpointCommand(Setpoint.kAlgaeDeliverBarge)
                                                        .withName("Deliver Barge"));
                }
        }

        private void setDefaultCommands() {
                // drivebase.setDefaultCommand(
                // Commands.parallel(
                // new GetNearestCoralStationPose(drivebase),
                // new GetNearestReefZonePose(drivebase, ls),
                // drivebase.driveCommand(
                // () -> -MathUtil.applyDeadband(
                // driverXbox.getLeftY()
                // * getAllianceFactor(),
                // OperatorConstants.LEFT_Y_DEADBAND),
                // () -> -MathUtil.applyDeadband(
                // driverXbox.getLeftX()
                // * getAllianceFactor(),

                // OperatorConstants.DEADBAND),
                // () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                // OperatorConstants.RIGHT_X_DEADBAND))));

                drivebase.setDefaultCommand(
                                Commands.parallel(
                                                new GetNearestCoralStationPose(drivebase),
                                                new GetNearestReefZonePose(drivebase, ls),
                                                new TeleopSwerve(drivebase,
                                                                () -> driverXbox.getLeftY()
                                                                                * getAllianceFactor(),

                                                                () -> driverXbox.getLeftX()
                                                                                * getAllianceFactor(),

                                                                () -> driverXbox.getRightX(),
                                                                () -> correctAngle)));

                elevator.setDefaultCommand(new PositionHoldElevatorPID(elevator, arm));

                arm.setDefaultCommand(new PositionHoldArmPID(arm));

                preIn.setDefaultCommand(preIn.positionCommand());

        }

        private void configureDriverBindings() {

                driverXbox.a().onTrue(Commands.runOnce(() -> correctAngle = !correctAngle));

                driverXbox.b().onTrue(cf.deliverToBargeWithArmCommand().withName("Deliver Algae Barge"));

                driverXbox.x().onTrue(gamepieces.deliverAlgaeToProcessorCommand().withName("Deliver Algae Processor"));

                driverXbox.y().onTrue(new DetectAlgaeWhileIntaking(gamepieces).withName("Intake Algae"));

                driverXbox.back().onTrue(drivebase.centerModulesCommand().withName("Center Modules"));

                driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance).withName("Zero Gyro"));

                driverXbox.leftTrigger().onTrue(
                                Commands.sequence(
                                                new IntakeCoralToSwitch(gamepieces, arm, false)
                                                                .withName("IntakeCoral"),
                                                new BackupOffSwitchAfterCoraIntake(gamepieces)));

                driverXbox.rightTrigger().onTrue(gamepieces.deliverCoralCommand().withName("Deliver Coral"));

                driverXbox.leftBumper().debounce(.1).and(driverXbox.rightBumper().negate()).whileTrue(
                                Commands.defer(() -> Commands.sequence(
                                                drivebase.setSide(Side.LEFT),
                                                llv.setPOILeft(),
                                                new PIDDriveToPose(drivebase, drivebase.reefTargetPose)),
                                                Set.of(drivebase)))
                                .onFalse(Commands.sequence(
                                                drivebase.setSide(Side.CENTER),
                                                llv.clearPOI()).withName("Left Reef PID"));

                driverXbox.rightBumper().debounce(.1).and(driverXbox.leftBumper().negate()).whileTrue(
                                Commands.defer(() -> Commands.sequence(
                                                drivebase.setSide(Side.RIGHT),
                                                llv.setPOIRight(),
                                                new PIDDriveToPose(drivebase, drivebase.reefTargetPose)),
                                                Set.of(drivebase)))
                                .onFalse(Commands.sequence(
                                                drivebase.setSide(Side.CENTER),
                                                llv.clearPOI()).withName("Right Reef PID"));

                driverXbox.rightBumper().and(driverXbox.leftBumper()).whileTrue(
                                Commands.defer(() -> Commands.sequence(
                                                drivebase.setSide(Side.CENTER),
                                                llv.clearPOI(),
                                                new PIDDriveToPose(drivebase, drivebase.reefTargetPose)),
                                                Set.of(drivebase)).withName("Center Reef PID"));

                // driverXbox.povDown().whileTrue(new DriveToNearestCoralStation(drivebase));

                driverXbox.povDown()
                                .onTrue(Commands.none());
                driverXbox.povUp().onTrue(Commands.runOnce(() -> arm.setGoalDegrees(-90)));

                driverXbox.povLeft().onTrue(Commands.runOnce(() -> preIn.setGoalDegreesCommand(45)));

                driverXbox.povRight().onTrue(Commands.runOnce(() -> preIn.setGoalDegreesCommand(0)));
        }

        public void configureCoDriverTeleopBindings() {

                coDriverXbox = new CommandXboxController(1);

                coDriverXbox.a().onTrue(cf.setSetpointCommand(Setpoint.kLevel1).withName("Set L1"));

                coDriverXbox.b().onTrue(cf.setSetpointCommand(Setpoint.kLevel2).withName("Set L2"));

                coDriverXbox.x().onTrue(cf.setSetpointCommand(Setpoint.kLevel3).withName("Set L3"));

                coDriverXbox.y().onTrue(cf.setSetpointCommand(Setpoint.kLevel4).withName("Set L4"));

                coDriverXbox.povDown()
                                .onTrue(cf.setSetpointCommand(Setpoint.kProcessorDeliver).withName("Set Processor"));

                coDriverXbox.povRight()
                                .onTrue(cf.setSetpointCommand(Setpoint.kAlgaePickUpL3).withName("Set Algae Pickup L3"));
                // cf.setSetpointCommand(Setpoint.kCoralStation).withName("Set Coral Station"));

                coDriverXbox.povLeft()
                                .onTrue(cf.setSetpointCommand(Setpoint.kAlgaePickUpL2).withName("Set Algae Pickup L2"));

                coDriverXbox.povUp()
                                .onTrue(preIn.setGoalDegreesCommand(0));

                coDriverXbox.rightBumper().onTrue(cf.homeElevatorAndArm().withName("Home Elevator Arm"));

                coDriverXbox.leftBumper().onTrue(
                                cf.setSetpointCommand(Setpoint.kAlgaeDeliverBarge).withName("Set Algae Barge"));

                coDriverXbox.leftTrigger().whileTrue(
                                Commands.defer(() -> gamepieces
                                                .jogCoralIntakeMotorsCommand(() -> coDriverXbox.getLeftY()),
                                                Set.of(gamepieces)))
                                .onFalse(gamepieces.stopGamepieceMotorsCommand());

                coDriverXbox.back()
                                .onTrue(Commands.parallel(
                                                preIn.setGoalDegreesCommand(90),
                                                Commands.runOnce(() -> arm.setGoalDegrees(-90))));

                coDriverXbox.rightTrigger().whileTrue(climber.jogClimberCommand(() -> coDriverXbox.getLeftX()))
                                .onFalse(Commands.runOnce(() -> climber.stop()));

                // (preIn.setGoalDegreesCommand(90));

                // coDriverXbox.start().onTrue(Commands.parallel(elevator.clearStickyFaultsCommand(),
                // arm.clearStickyFaultsCommand(), gamepieces.clearStickyFaultsCommand()));
                coDriverXbox.start().onTrue(preIn.setGoalDegreesCommand(10));

        }

        public void configureCoDriverTestBindings() {

                coDriverXbox = new CommandXboxController(1);

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

                // coDriverXbox.leftTrigger().whileTrue(new JogClimber(climber, coDriverXbox))
                // .onFalse(Commands.runOnce(() -> climber.stop()));

                // coDriverXbox.y().onTrue(
                // elevator.setGoalInchesCommand(ElevatorSetpoints.kHome));

                // coDriverXbox.a().onTrue(elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel2));

                // coDriverXbox.b().onTrue(
                // elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel4));

                // coDriverXbox.x().onTrue(
                // elevator.setGoalInchesCommand(ElevatorSetpoints.kLevel3));

                // SYS ID ElEVATOR TESTS
                coDriverXbox.y().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));

                coDriverXbox.a().whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                coDriverXbox.b().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

                coDriverXbox.x().whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                coDriverXbox.povLeft()
                                .onTrue(preIn.setGoalDegreesCommand(20));

                coDriverXbox.povUp()
                                .onTrue(preIn.setGoalDegreesCommand(0));

                coDriverXbox.povDown()
                                .onTrue(preIn.setGoalDegreesCommand(40));

                coDriverXbox.povRight()
                                .onTrue(preIn.setGoalDegreesCommand(90));

                coDriverXbox.leftStick().onTrue(gamepieces.deliverCoralCommand());
        }

        private void buildAutoChooser() {
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);
        }

        private void printCommandEvents() {
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

        private void setTriggerActions() {

                // coralAtIntake.onTrue(Commands.parallel(
                // Commands.runOnce(() -> arm.setGoalDegrees(ArmSetpoints.kTravel)),
                // rumble(driverXbox, RumbleType.kRightRumble, 1)));

                stickyFaulTrigger.onTrue(rumble(coDriverXbox, RumbleType.kBothRumble, 1));

                reefZoneChange.onTrue(rumble(driverXbox, RumbleType.kLeftRumble, .25))
                                .onTrue(Commands.runOnce(() -> drivebase.reefZoneLast = drivebase.reefZone));

                coralAtIntake.onTrue(ls.getCoralIntakeLEDsCommand())
                        .onFalse(ls.getCoralDeliverLEDsCommand());
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

        // public void setDriveMode() {
        // configureBindings();
        // }

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
