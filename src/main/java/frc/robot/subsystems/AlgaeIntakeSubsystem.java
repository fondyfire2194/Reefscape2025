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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Factories.CommandFactory.AlgaeSetpoints;

public class AlgaeIntakeSubsystem extends SubsystemBase {

  public SparkMax algaeintakeMotor;
  public SparkClosedLoopController algaeintakeController;
  private SparkMaxConfig algaeintakeConfig;

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
    algaeintakeMotor = new SparkMax(Constants.CANIDConstants.algaeintakeID, MotorType.kBrushless);
    algaeintakeController = algaeintakeMotor.getClosedLoopController();

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

    algaeintakeMotor.configure(algaeintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    algaeintakeMotor.stopMotor();
    algaeintakeController.setReference(0, ControlType.kVelocity);
  }

  public Command stopIntakeCommand() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(AlgaeSetpoints.kStop)),
        Commands.runOnce(() -> targetRPM = AlgaeSetpoints.kStop));
  }

  public double getRPM() {
    if (RobotBase.isReal())
      return algaeintakeMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public Command intakeAlgaeCommand() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(AlgaeSetpoints.kReefPickUpL123)),
        Commands.runOnce(() -> targetRPM = AlgaeSetpoints.kReefPickUpL123));
  }

  public Command deliverAlgaeCommand() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(AlgaeSetpoints.kDiliver)),
        Commands.runOnce(() -> targetRPM = AlgaeSetpoints.kDiliver));
  }

  private void runAtVelocity(double rpm) {
    algaeintakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(reverseRPM.in(RPM));
  }

  public double getAmps() {
    return algaeintakeMotor.getOutputCurrent();
  }

  public boolean getStickyFaults() {
    return algaeintakeMotor.hasActiveFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> algaeintakeMotor.clearFaults());
  }

}
