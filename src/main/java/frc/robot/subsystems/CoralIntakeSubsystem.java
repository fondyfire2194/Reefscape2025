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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Factories.CommandFactory.AlgaeSetpoints;
import frc.robot.Factories.CommandFactory.CoralSetpoints;

public class CoralIntakeSubsystem extends SubsystemBase {

  public SparkMax coralIntakeMotor;
  public SparkClosedLoopController coralintakeController;
  SparkMaxConfig coralintakeConfig;

  public SparkLimitSwitch coralDetectSwitch;

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

    coralIntakeMotor = new SparkMax(Constants.CANIDConstants.coralintakeID, MotorType.kBrushless);

    coralintakeController = coralIntakeMotor.getClosedLoopController();
    coralDetectSwitch = coralIntakeMotor.getForwardLimitSwitch();
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

    coralintakeConfig.signals.primaryEncoderPositionPeriodMs(5);

    coralIntakeMotor.configure(coralintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    runAtVelocity(0);
    coralIntakeMotor.stopMotor();
  }

  public Command stopCoralIntakeCommand() {
    return Commands.runOnce(() -> stopMotor(), this);
  }

  public void coralintakeToSwitch(double RPM) {
    // enableLimitSwitch(true);
    runAtVelocity(RPM);
  }

  public Command deliverCoralCommandL123() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(CoralSetpoints.kReefPlaceL123)),
        Commands.runOnce(() -> targetRPM = CoralSetpoints.kReefPlaceL123));
  }

  public Command deliverCoralCommandL4() {
    return Commands.parallel(Commands.runOnce(() -> runAtVelocity(CoralSetpoints.kReefPlaceL4)),
        Commands.runOnce(() -> targetRPM = CoralSetpoints.kReefPlaceL4));
  }

  public Command coralintakeToSwitchCommand() {
    return Commands.parallel(
        Commands.run(() -> coralintakeToSwitch(CoralSetpoints.kFeederStation))
            // .until(() -> coralAtIntake())
            .withTimeout(coralAtSwitchTime)
            .andThen(stopCoralIntakeCommand()),
        Commands.runOnce(() -> targetRPM = CoralSetpoints.kFeederStation));
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral/RPM", getRPM());
  }

  public void enableLimitSwitch(boolean enable) {
    coralintakeConfig.limitSwitch.forwardLimitSwitchEnabled(enable);
  }

  public boolean getLimitSwitchEnabled() {
    return coralIntakeMotor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
  }

  public double getAmps() {
    return coralIntakeMotor.getOutputCurrent();
  }

  public double getRPM() {
    if (RobotBase.isReal())
      return coralIntakeMotor.getEncoder().getVelocity();
    else
      return targetRPM;
  }

  public boolean getActiveFault() {
    return coralIntakeMotor.hasActiveFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> coralIntakeMotor.clearFaults());
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public double getPosition() {
    return coralIntakeMotor.getEncoder().getPosition();
  }

  public double getVelocity() {
    return coralIntakeMotor.getEncoder().getVelocity();
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < 200;
  }

}
