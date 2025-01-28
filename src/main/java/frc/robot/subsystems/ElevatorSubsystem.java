// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private double maxMotorRPM = 5700;

  public final double kElevatorGearing = (62 / 9) * (22 / 16);// 9.4722222

  // assume 10:1 gearing 50 tooth pulley 2mm pitch 1 rev pulley =100mm = 1 rev
  // motor
  // = 10mm
  public double positionConversionFactor = 1 / kElevatorGearing;
  public double velocityConversionFactor = positionConversionFactor / 60;

  public double maxVelocityMPS = maxMotorRPM * velocityConversionFactor;// 570/60 = 95

  public final double elevatorKp = .005;
  public final double elevatorKi = 0;
  public final double elevatorKd = 0;

  public final double kCarriageMass = Units.lbsToKilograms(1); // kg

  public final double kElevatorDrumRadius = Units.inchesToMeters(1.757 / 2);

  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public final double minElevatorHeightMeters = 0.25;
  public final double maxElevatorHeightMeters = 2.8;//

  public final SparkMax m_leftMotor = new SparkMax(CANIDConstants.leftElevatorID, MotorType.kBrushless);

  private SparkClosedLoopController leftClosedLoopController = m_leftMotor.getClosedLoopController();

  public final SparkMax m_rightMotor = new SparkMax(CANIDConstants.rightElevatorID, MotorType.kBrushless);

  private SparkClosedLoopController rightClosedLoopController = m_leftMotor.getClosedLoopController();

  public static double TRAJECTORY_VEL = 20;
  public static double TRAJECTORY_ACCEL = 20;
  public static double drumRadius = .164;// meters

  public int posrng;

  // private boolean shutDownElevatorPositioning;

  public boolean atUpperLimit;

  public boolean atLowerLimit;

  public double elevatorCurrentTarget;

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {

    SmartDashboard.putNumber("Elevator/posconv", positionConversionFactor);
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.01)
        .outputRange(-1, 1).maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(50)
        .maxAcceleration(100)
        .allowedClosedLoopError(0.5);

    leftConfig.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

    rightConfig
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control
        .p(0.1)
        .outputRange(-1, 1).maxMotion
        // Set MAXMotion parameters for position control
        .maxVelocity(4200)
        .maxAcceleration(6000)
        .allowedClosedLoopError(0.5);

    rightConfig.encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor);

    m_leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    resetPosition(0);
    setTargetMeters(minElevatorHeightMeters);
  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Elevator/posiiton", getLeftPositionMeters());
    SmartDashboard.putNumber("Elevator/Vel", getLeftVelocity());
    SmartDashboard.putNumber("Elevator/APPO", m_leftMotor.getAppliedOutput());

  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getLeftPositionInches() {
    return Units.metersToInches(getLeftPositionMeters());
  }

  public double getRightPositionInches() {
    return Units.metersToInches(getLeftPositionMeters());
  }

  public double getLeftPositionMeters() {
    return minElevatorHeightMeters + m_leftMotor.getEncoder().getPosition();
  }

  public double getRightPositionMeters() {
    return minElevatorHeightMeters + m_leftMotor.getEncoder().getPosition();
  }

  public double getLeftVelocity() {
    return m_rightMotor.getEncoder().getVelocity();
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
    m_leftMotor.setVoltage(0.0);
    m_rightMotor.setVoltage(0.0);

  }

  public void position() {

    posrng++;
    leftClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    rightClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void resetPosition(double val) {
    m_leftMotor.getEncoder().setPosition(val);
    m_rightMotor.getEncoder().setPosition(val);

  }

  @Override
  public void periodic() {
    atUpperLimit = getLeftPositionMeters() >= maxElevatorHeightMeters
        || getRightPositionMeters() >= maxElevatorHeightMeters;

    atLowerLimit = getLeftPositionMeters() <= minElevatorHeightMeters
        || (getRightPositionMeters() <= minElevatorHeightMeters
            && RobotBase.isReal());

    SmartDashboard.putBoolean("Elevator/onUpperLimit", atUpperLimit);
    SmartDashboard.putBoolean("Elevator/onLowerLimit", atLowerLimit);

  }

  public void setTargetInches(double inches) {
    elevatorCurrentTarget = Units.inchesToMeters(inches);
    SmartDashboard.putNumber("Elevator/current target", elevatorCurrentTarget);
  }

  public void setTargetMeters(double meters) {
    elevatorCurrentTarget = meters;

  }

}