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

  public SparkLimitSwitch coralDetectSwitch;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  @Log(key = "target rpm")
  public double targetRPM;

  public final double coralintakeKp = .00002; // P gains caused oscilliation
  public final double coralintakeKi = 0.0;
  public final double coralintakeKd = 0.00;
  public final double coralintakeKFF = .95 / 11000;

  private double coralAtSwitchTime = 3;

  private double lockAlgaeSet = .01;
  private int lockAlgaeAmps = 2;

  /** Creates a new coralintake. */
  public GamepieceSubsystem() {

    if (RobotBase.isReal())
      coralAtSwitchTime = 10;
    else
      coralAtSwitchTime = 1;

    gamepieceMotor = new SparkMax(Constants.CANIDConstants.gamepieceID, MotorType.kBrushless);

    gamepieceController = gamepieceMotor.getClosedLoopController();
    coralDetectSwitch = gamepieceMotor.getForwardLimitSwitch();
    gamepieceConfig = new SparkMaxConfig();

    gamepieceConfig
        .inverted(false)
        .smartCurrentLimit(20, 20)
        .idleMode(IdleMode.kBrake);

    gamepieceConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    gamepieceConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(coralintakeKFF)
        .pid(coralintakeKp, coralintakeKi, coralintakeKd);

    gamepieceConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    gamepieceConfig.signals.primaryEncoderPositionPeriodMs(10);

    gamepieceMotor.configure(gamepieceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    disableLimitSwitch();

   
  }

  public void setCurrentLimit(int amps) {
    gamepieceConfig.smartCurrentLimit(amps);
  }

  public void lockMotor() {
    gamepieceMotor.set(lockAlgaeSet);
    setCurrentLimit(lockAlgaeAmps);
  }

  public void stopMotor() {
    runAtVelocity(0);
    targetRPM = 0;
    gamepieceMotor.stopMotor();
  }

  public Command stopMotorCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public void coralintakeToSwitch(double RPM) {
    enableLimitSwitch();
    runAtVelocity(RPM);
  }

  public Command deliverCoralCommandL123() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> runAtVelocity(CoralRPMSetpoints.kReefPlaceL123)),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL123));
  }

  public Command deliverCoralCommandL4() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> runAtVelocity(CoralRPMSetpoints.kReefPlaceL4)),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL4));
  }

  public Command intakeCoralToSwitchCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> enableLimitSwitch()),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kCoralStation),
        Commands.run(() -> coralintakeToSwitch(CoralRPMSetpoints.kCoralStation))
            .until(() -> coralAtIntake())
            .withTimeout(coralAtSwitchTime)
            .andThen(stopMotorCommand()));
  }

  public Command intakeAlgaeCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> disableLimitSwitch()),
        new DetectAlgaeWhileIntaking(this));
  }

  public Command deliverAlgaeCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> disableLimitSwitch()),
        Commands.runOnce(() -> runAtVelocity(AlgaeRPMSetpoints.kProcessorDeliver)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kProcessorDeliver));
  }

  public void run(double speed) {
    gamepieceMotor.set(speed);
  }

  public void runAtVelocity(double rpm) {
    setCurrentLimit(20);
    if (RobotBase.isReal())
      gamepieceController.setReference(rpm, ControlType.kVelocity);
  }

  public Command runAtVelocityCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> enableLimitSwitch()),
        Commands.runOnce(() -> runAtVelocity(targetRPM)));
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
    SmartDashboard.putNumber("Gamepieca/Velocity", gamepieceMotor.getEncoder().getVelocity());
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

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

  public Command jogMotorCommand(DoubleSupplier speed) {
  
    return Commands.run(() -> gamepieceMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
  }
}
