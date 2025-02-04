// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Factories.CommandFactory.CoralRPMSetpoints;
import monologue.Annotations.Log;
import monologue.Logged;

public class CoralIntakeSubsystem extends SubsystemBase implements Logged {

  public SparkMax coralMotor;
  public SparkClosedLoopController coralintakeController;
  SparkMaxConfig coralintakeConfig;

  public SparkLimitSwitch coralDetectSwitch;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  public double targetRPM;

  public final double coralintakeKp = .00002; // P gains caused oscilliation
  public final double coralintakeKi = 0.0;
  public final double coralintakeKd = 0.00;
  public final double coralintakeKFF = .95 / 11000;

  private double coralAtSwitchTime = 3;

  /** Creates a new coralintake. */
  public CoralIntakeSubsystem() {

    if (RobotBase.isReal())
      coralAtSwitchTime = 3;
    else
      coralAtSwitchTime = 5;

    coralMotor = new SparkMax(Constants.CANIDConstants.coralintakeID, MotorType.kBrushless);

    coralintakeController = coralMotor.getClosedLoopController();
    coralDetectSwitch = coralMotor.getForwardLimitSwitch();
    coralintakeConfig = new SparkMaxConfig();

    coralintakeConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    coralintakeConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    coralintakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(coralintakeKFF)
        .pid(coralintakeKp, coralintakeKi, coralintakeKd);

    coralintakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    coralintakeConfig.signals.primaryEncoderPositionPeriodMs(10);

    coralMotor.configure(coralintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    runAtVelocity(0);
    targetRPM = 0;
    coralMotor.stopMotor();
  }

  public Command stopCoralMotorCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public void coralintakeToSwitch(double RPM) {
    enableLimitSwitch(true);
    runAtVelocity(RPM);
  }

  public Command deliverCoralCommandL123() {
    return Commands.parallel(
        Commands.runOnce(() -> enableLimitSwitch(false)),
        Commands.runOnce(() -> runAtVelocity(CoralRPMSetpoints.kReefPlaceL123)),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL123));
  }

  public Command deliverCoralCommandL4() {
    return Commands.parallel(
        Commands.runOnce(() -> enableLimitSwitch(false)),
        Commands.runOnce(() -> runAtVelocity(CoralRPMSetpoints.kReefPlaceL4)),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kReefPlaceL4));
  }

  public Command coralintakeToSwitchCommand() {
    return Commands.parallel(
        Commands.run(() -> coralintakeToSwitch(CoralRPMSetpoints.kCoralStation))
            .until(() -> coralAtIntake())
            .withTimeout(coralAtSwitchTime)
            .andThen(stopCoralMotorCommand()),
        Commands.runOnce(() -> targetRPM = CoralRPMSetpoints.kCoralStation));
  }

  public void runAtVelocity(double rpm) {
    if (RobotBase.isReal())
      coralintakeController.setReference(rpm, ControlType.kVelocity);
  }

  public Command runAtVelocityCommand() {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> enableLimitSwitch(true)),
        Commands.runOnce(() -> runAtVelocity(targetRPM)));
  }

  public Command setTargetRPM(double rpm) {
    return Commands.runOnce(() -> targetRPM = rpm);
  }

  public boolean coralAtIntake() {
    return coralDetectSwitch.isPressed();
  }

  @Override
  public void periodic() {
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());
  }

  public void enableLimitSwitch(boolean enable) {
    coralintakeConfig.limitSwitch.forwardLimitSwitchEnabled(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return coralMotor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
  }

  @Log(key = "coral amps")
  public double getAmps() {
    return coralMotor.getOutputCurrent();
  }

  @Log(key = "coral rpm")
  public double getRPM() {
    if (RobotBase.isReal())
      return coralMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  public boolean getActiveFault() {
    return coralMotor.hasActiveFault();
  }

  public boolean getStickyFault() {
    return coralMotor.hasStickyFault();
  }

  public boolean getWarnings() {
    return coralMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> coralMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return coralMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return coralMotor.getEncoder().getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

  public Command jogMotorCommand(double speed) {
    return Commands.runOnce(() -> coralMotor.setVoltage(speed * RobotController.getBatteryVoltage()));
  }
}
