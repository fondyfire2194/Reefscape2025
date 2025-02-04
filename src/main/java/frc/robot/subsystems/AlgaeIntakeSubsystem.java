// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Factories.CommandFactory.AlgaeRPMSetpoints;
import monologue.Annotations.Log;
import monologue.Logged;

public class AlgaeIntakeSubsystem extends SubsystemBase implements Logged {

  public SparkMax algaeMotor;
  public SparkClosedLoopController algaeintakeController;
  private SparkMaxConfig algaeintakeConfig;

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

  public final double maxIntakeMotorRPM = 5700;
  public final double algaeintakeConversionVelocityFactor = 1;
  public final double algaeintakeConversionPositionFactor = 1;
  public final double voltageComp = 12;

  public final int algaeintakeContinuousCurrentLimit = 60;
  public double jogSpeed = 1;
  public AngularVelocity reverseRPM = RPM.of(-500);
  public double reverseTime = 2;
  public double noteInIntakeAmps = 30;
  public final AngularVelocity algaeintakeSpeed = RPM.of(4500);
  public final double algaeintakeKp = 2e-4;
  public final double algaeintakeKi = 0.0;
  public final double algaeintakeKd = 0.00;
  public final double algaeintakeKFF = .95 / maxIntakeMotorRPM;

  private double targetRPM;

  /** Creates a new Intake. */
  public AlgaeIntakeSubsystem() {
    algaeMotor = new SparkMax(Constants.CANIDConstants.algaeintakeID, MotorType.kBrushless);
    algaeintakeController = algaeMotor.getClosedLoopController();

    algaeintakeConfig = new SparkMaxConfig();

    algaeintakeConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    algaeintakeConfig.encoder
        .positionConversionFactor(algaeintakeConversionPositionFactor)
        .velocityConversionFactor(algaeintakeConversionVelocityFactor);

    algaeintakeConfig.closedLoop
        .velocityFF(algaeintakeKFF)
        .pid(algaeintakeKp, algaeintakeKi, algaeintakeKd);

    algaeintakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    algaeintakeConfig.signals.primaryEncoderPositionPeriodMs(5);

    algaeMotor.configure(algaeintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    algaeMotor.stopMotor();
    targetRPM=0;
    algaeintakeController.setReference(0, ControlType.kVelocity);
  }

  public Command stopMotorCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> runAtVelocity(AlgaeRPMSetpoints.kStop)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kStop));
  }

  public double getRPM() {
    if (RobotBase.isReal())
      return algaeMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  public boolean getActiveFault() {
    return algaeMotor.hasActiveFault();
  }

  public boolean getStickyFault() {
    return algaeMotor.hasStickyFault();
  }

  public boolean getWarnings() {
    return algaeMotor.hasActiveWarning();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());

  }

  public Command intakeAlgaeCommand() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(AlgaeRPMSetpoints.kReefPickUpL123)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kReefPickUpL123));
  }

  public Command deliverAlgaeCommand() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(AlgaeRPMSetpoints.kDiliver)),
        Commands.runOnce(() -> targetRPM = AlgaeRPMSetpoints.kDiliver));
  }

  private void runAtVelocity(double rpm) {
    algaeintakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(reverseRPM.in(RPM));
  }

  public double getAmps() {
    return algaeMotor.getOutputCurrent();
  }

  public boolean getStickyFaults() {
    return algaeMotor.hasActiveFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> algaeMotor.clearFaults());
  }

  public Command jogMotorCommand(double speed) {
    return Commands.runOnce(() -> algaeMotor.setVoltage(speed * RobotController.getBatteryVoltage()));
  }

}
