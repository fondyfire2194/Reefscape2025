// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory.AlgaeRPMSetpoints;
import frc.robot.Factories.CommandFactory.CoralRPMSetpoints;
import frc.robot.commands.Gamepieces.DetectAlgaeWhileIntaking;
import monologue.Annotations.Log;
import monologue.Logged;

public class GamepieceSubsystem extends SubsystemBase implements Logged {

  public SparkMax gamepieceMotor;
  public SparkClosedLoopController gamepieceController;
  SparkMaxConfig gamepieceConfig;

  public SparkMax coralIntakeMotor;
  public SparkClosedLoopController coralIntakeController;
  SparkMaxConfig coralIntakeConfig;

  public SparkLimitSwitch coralDetectSwitch;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  @Log(key = "target rpm")
  public double targetRPM;

  public final double gamepieceKp = .00002; // P gains caused oscilliation
  public final double gamepieceKi = 0.0;
  public final double gamepieceKd = 0.00;
  public final double gamepieceKFF = .95 / 11000;

  public final double coralIntakeKp = .00002; // P gains caused oscilliation
  public final double coralIntakeKi = 0.0;
  public final double coralIntakeKd = 0.00;
  public final double coralIntakeKFF = .95 / 11000;

  private double coralAtSwitchTime = 10;

  private double lockAlgaeSet = .01;
  private int lockAlgaeAmps = 2;
  private int inOutAlgaeAmps = 20;

  /** Creates a new gamepiece. */
  public GamepieceSubsystem() {

    if (RobotBase.isReal())
      coralAtSwitchTime = 10;
    else
      coralAtSwitchTime = 1;

    gamepieceMotor = new SparkMax(Constants.CANIDConstants.gamepieceID, MotorType.kBrushless);
    gamepieceController = gamepieceMotor.getClosedLoopController();
    coralDetectSwitch = gamepieceMotor.getForwardLimitSwitch();
    gamepieceConfig = new SparkMaxConfig();

    coralIntakeMotor = new SparkMax(Constants.CANIDConstants.coralIntakeID, MotorType.kBrushless);
    coralIntakeController = gamepieceMotor.getClosedLoopController();
    coralIntakeConfig = new SparkMaxConfig();

    gamepieceConfig
        .inverted(true)
        .smartCurrentLimit(20, 20)
        .idleMode(IdleMode.kBrake);

    gamepieceConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    gamepieceConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(gamepieceKFF)
        .pid(gamepieceKp, gamepieceKi, gamepieceKd);

    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    gamepieceConfig.signals.primaryEncoderPositionPeriodMs(10);

    gamepieceMotor.configure(gamepieceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    disableLimitSwitch();

    coralIntakeConfig
        .inverted(true)
        .smartCurrentLimit(20, 20)
        .idleMode(IdleMode.kBrake);

    coralIntakeConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    coralIntakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(coralIntakeKFF)
        .pid(coralIntakeKp, coralIntakeKi, coralIntakeKd);

    coralIntakeConfig.signals.primaryEncoderPositionPeriodMs(10);

    coralIntakeMotor.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCurrentLimit(int amps) {
    gamepieceConfig.smartCurrentLimit(amps);
  }

  public void lockMotor() {
    gamepieceMotor.set(lockAlgaeSet);
    setCurrentLimit(lockAlgaeAmps);
  }

  public void stopGamepieceMotor() {
    runGamepieceMotorAtVelocity(0);
    targetRPM = 0;
    gamepieceMotor.stopMotor();
  }

  public void stopCoralIntakeMotor() {
    runCoralIntakeMotorAtVelocity(0);
    targetRPM = 0;
    gamepieceMotor.stopMotor();
  }

  public Command stopGamepieceMotorsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> stopCoralIntakeMotor()),
        Commands.runOnce(() -> stopGamepieceMotor()));
  }

  public void coralintakeToSwitch(double RPM, double CIRPM) {
    enableLimitSwitch();
    runGamepieceMotorAtVelocity(RPM);
    runCoralIntakeMotorAtVelocity(CIRPM);
  }

  
  public Command deliverCoralCommandL123() {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> disableLimitSwitch()),
            Commands.runOnce(() -> runGamepieceMotorAtVelocity(CoralRPMSetpoints.kReefPlaceL123)),
            Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL123)),
        new WaitCommand(3),
        stopGamepieceMotorsCommand());
  }

  public Command deliverCoralCommandL4() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> runGamepieceMotorAtVelocity(CoralRPMSetpoints.kReefPlaceL4)),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL4))
        .withTimeout(5);
  }

  public Command intakeCoralToSwitchCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> enableLimitSwitch()),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kCoralStation),
        Commands.run(() -> coralintakeToSwitch(CoralRPMSetpoints.kCoralStation, CoralRPMSetpoints.kCoralIntakeMotorRPM))
            .until(() -> coralAtIntake())
            .withTimeout(coralAtSwitchTime)
            .andThen(stopGamepieceMotorsCommand()));
  }

  public Command intakeAlgaeCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> disableLimitSwitch()),
        new DetectAlgaeWhileIntaking(this));
  }

  public Command deliverAlgaeToProcessorCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> setCurrentLimit(inOutAlgaeAmps)),
        Commands.runOnce(() -> runGamepieceMotorAtVelocity(AlgaeRPMSetpoints.kProcessorDeliver)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kProcessorDeliver));
  }

  public Command deliverAlgaeToBargeCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> setCurrentLimit(inOutAlgaeAmps)),
        Commands.runOnce(() -> runGamepieceMotorAtVelocity(AlgaeRPMSetpoints.kBargeDeliver)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kBargeDeliver));
  }

  public void run(double speed) {
    gamepieceMotor.set(speed);
  }

  public void runGamepieceMotorAtVelocity(double rpm) {
    setCurrentLimit(20);
    if (RobotBase.isReal())
      gamepieceController.setReference(rpm, ControlType.kVelocity);
  }

  public void runCoralIntakeMotorAtVelocity(double rpm) {
    setCurrentLimit(20);
    if (RobotBase.isReal())
      coralIntakeController.setReference(rpm, ControlType.kVelocity);
  }

  public Command setTargetRPM(double rpm) {
    return Commands.runOnce(() -> targetRPM = rpm);
  }

  @Log(key = "coral at intake")
  public boolean coralAtIntake() {
    return coralDetectSwitch.isPressed();
  }

  @Override
  public void periodic() {
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());
    SmartDashboard.putNumber("Gamepiece/GPVelocity", gamepieceMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Gamepiece/INTVelocity", coralIntakeMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("Gamepiece/GPAmps", gamepieceMotor.getOutputCurrent());
    SmartDashboard.putNumber("Gamepiece/INTAmps", coralIntakeMotor.getOutputCurrent());

  }

  public void enableLimitSwitch() {
    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(true);
  }

  public void disableLimitSwitch() {
    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(false);
  }

  public boolean getLimitSwitchEnabled() {
    return gamepieceMotor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
  }

  @Log(key = "gamepiece amps")
  public double getAmps() {
    return gamepieceMotor.getOutputCurrent();
  }

  @Log(key = "gamepiece rpm")
  public double getRPM() {
    if (RobotBase.isReal())
      return gamepieceMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  @Log(key = "fault")
  public boolean getActiveFault() {
    return gamepieceMotor.hasActiveFault();
  }

  @Log(key = "sticky fault")
  public boolean getStickyFault() {
    return gamepieceMotor.hasStickyFault();
  }

  @Log(key = "warning")
  public boolean getWarnings() {
    return gamepieceMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> gamepieceMotor.clearFaults());
  }

  public double getPosition() {
    return gamepieceMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return gamepieceMotor.getEncoder().getVelocity();
  }

  public double getGamepieceAmps() {
    return gamepieceMotor.getOutputCurrent();
  }

  public double getIntakeAmps() {
    return coralIntakeMotor.getOutputCurrent();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

  public Command jogGamepieceMotorCommand(DoubleSupplier speed) {
    return Commands.run(() -> gamepieceMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
  }

  public Command jogCoralIntakeMotorsCommand(DoubleSupplier speed) {
    return Commands.parallel(
        Commands.run(() -> gamepieceMotor.setVoltage(-speed.getAsDouble() * RobotController.getBatteryVoltage())),
        Commands.run(() -> coralIntakeMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage())));
  }
}
