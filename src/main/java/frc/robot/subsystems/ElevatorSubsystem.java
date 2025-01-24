// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // This gearbox represents a gearbox containing 1 Neo
  

  public final double kElevatorKp = 5;
  public final double kElevatorKi = 0;
  public final double kElevatorKd = 0;

  public final double kElevatorkS = 0.0; // volts (V)
  public final double kElevatorkG = 0.762; // volts (V)
  public final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

  public final static double kElevatorGearing = 10.0;
  public final static double kElevatorDrumRadius = Units.inchesToMeters(2.0);

  public final double UPPER_POSITION_LIMIT = 0;

  public final double LOWER_POSITION_LIMIT = 0;
  public final double kCarriageMass = 4.0; // kg

  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public final double kMinElevatorHeightMeters = 0.0;
  public final double kMaxElevatorHeightMeters = 10.25;

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
      kElevatorkS,
      kElevatorkG,
      kElevatorkV,
      kElevatorkA);

  double elevatorKp;
  double elevatorKi;
  double elevatorKd;

  public final SparkMax m_leftMotor = new SparkMax(CANIDConstants.leftElevatorID, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();

  public final SparkMax m_rightMotor = new SparkMax(CANIDConstants.rightElevatorID, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

  public TrapezoidProfile.Constraints constraints;

  public ProfiledPIDController leftPidController;
  public ProfiledPIDController rightPidController;
  public static double TRAJECTORY_VEL = 20;
  public static double TRAJECTORY_ACCEL = 20;
  public static double positionConversionFactor = 2;
  public static double velocityConversionFactor = positionConversionFactor / 60;

  public int posrng;

  private double leftPidOut;

  private boolean shutDownElevatorPositioning;

  private double powerDownLimit;

  private double powerUpLimit;

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kElevatorKp, kElevatorKi, kElevatorKd)
        .outputRange(-1, 1).maxMotion
        .maxVelocity(10)// Elevator.convertDistanceToRotations(Meters.of(1)).per(Second).in(RPM))
        .maxAcceleration(20);
    leftConfig.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

    rightConfig
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kElevatorKp, kElevatorKi, kElevatorKd)
        .outputRange(-1, 1).maxMotion
        .maxVelocity(10)// Elevator.convertDistanceToRotations(Meters.of(1)).per(Second).in(RPM))
        .maxAcceleration(20);
    rightConfig.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);
    m_leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

   
    constraints = new TrapezoidProfile.Constraints(TRAJECTORY_VEL, TRAJECTORY_ACCEL);
    leftPidController = new ProfiledPIDController(elevatorKp, elevatorKi, elevatorKd, constraints);
    rightPidController = new ProfiledPIDController(elevatorKp, elevatorKi, elevatorKd, constraints);

  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic() {

  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getLeftPositionInches() {
    return m_leftEncoder.getPosition();
  }

  public double getRightPositionInches() {
    return m_rightEncoder.getPosition();
  }

  public double getAverageHeight() {
    return (getLeftPositionInches() + getRightPositionInches()) / 2;
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance) {
    return new Trigger(() -> MathUtil.isNear(height,
        getAverageHeight(),
        tolerance));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop() {
    m_leftMotor.set(0.0);
    m_rightMotor.set(0.0);

  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry() {

  }

  public void position() {

    posrng++;

    boolean elevatorHigh = getLeftPositionInches() >= kMaxElevatorHeightMeters
        || getRightPositionInches() >= kMaxElevatorHeightMeters;
    boolean elevatorLow = getLeftPositionInches() <= kMinElevatorHeightMeters
        || getRightPositionInches() <= kMinElevatorHeightMeters;

    leftPidOut = leftPidController.calculate(getLeftPositionInches());

    double leftPowerVal = clamp(leftPidOut, -powerDownLimit, powerUpLimit);

    if (!shutDownElevatorPositioning && (leftPowerVal > 0 && !elevatorHigh || leftPowerVal < 0 && !elevatorLow))
      m_leftMotor.set(leftPowerVal);
    else
      m_leftMotor.set(0);

    double rightPidOut = rightPidController.calculate(getRightPositionInches());

    double rightPowerVal = clamp(rightPidOut, -powerDownLimit, powerUpLimit);

    if (!shutDownElevatorPositioning && (rightPowerVal > 0 && !elevatorHigh || rightPowerVal < 0 && !elevatorLow))
      m_rightMotor.set(rightPowerVal);
    else
      m_rightMotor.set(0);
  }

  @Override
  public void periodic() {

    updateTelemetry();
  }

  public void setTargetInches(double temp) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTargetInches'");
  }

  /**
   * Returns value clamped between low and high boundaries.
   *
   * @param value Value to clamp.
   * @param low   The lower boundary to which to clamp value.
   * @param high  The higher boundary to which to clamp value.
   */
  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }


}