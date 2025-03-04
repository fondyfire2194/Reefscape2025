// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  public SparkMax climberMotor;
  public SparkClosedLoopController climberController;
  SparkMaxConfig climberConfig;

  public final double climberKp = .00002; // P gains caused oscilliation
  public final double climberKi = 0.0;
  public final double climberKd = 0.00;
  public final double climberKFF = .95 / 11000;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberMotor = new SparkMax(Constants.CANIDConstants.coralIntakeID, MotorType.kBrushless);
    climberController = climberMotor.getClosedLoopController();
    climberConfig = new SparkMaxConfig();

    climberConfig
        .inverted(false)
        .smartCurrentLimit(20, 20)
        .idleMode(IdleMode.kBrake);

    climberConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(climberKFF)
        .pid(climberKp, climberKi, climberKd);

    climberConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    climberConfig.signals.primaryEncoderPositionPeriodMs(10);

    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climber/Amps", getAmps());
    SmartDashboard.putNumber("Climber/Velocity", getRPM());

  }

  public Command runClimberMotorAtVelocityCommand(double rpm) {
    return Commands.run(() -> climberController.setReference(rpm, ControlType.kVelocity));
  }

  public double getAmps() {
    return climberMotor.getOutputCurrent();
  }

  public double getVolts() {
    return climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  public double getRPM() {
    return climberMotor.getEncoder().getVelocity();
  }


  public Command jogClimberCommand(DoubleSupplier speed) {
    return Commands.run(() -> climberMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
  }

}
