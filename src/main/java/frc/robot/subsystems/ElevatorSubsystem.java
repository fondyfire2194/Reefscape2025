// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {

  public final double kElevatorGearing = ((62. * 22.) / (9. * 16.));// 9.4722222

  public final double kElevatorDrumRadiusInches = 1.757 / 2;

  public final double kElevatorDrumRadiusMeters = Units.inchesToMeters(kElevatorDrumRadiusInches);

  public final int sprocketTeeth = 22;

  public final double metersPerSprocketRev = kElevatorDrumRadiusMeters * 2 * Math.PI;// 6.3 inches

  public final double meterspersecondsprocket = metersPerSprocketRev * (80. / kElevatorGearing);

  public double metersPerMotorRev = (metersPerSprocketRev / kElevatorGearing);//

  public double positionConversionFactor = metersPerMotorRev;
  public double velocityConversionFactor = positionConversionFactor / 60;

  public double elevatorToGround = 6;

  public double maxVelocityMPS = meterspersecondsprocket;

  public final double elevatorKp = .95;
  public final double elevatorKi = 0;
  public final double elevatorKd = 0;

  public final double elevatorKs = .06;
  public final double elevatorKg = 1.1;
  public final double elevatorKv = 10;
  public final double elevatorKa = 0.08;

  public final double kCarriageMass = Units.lbsToKilograms(10); // kg

  public final Distance minElevatorHeight = Inches.of(5);
  public final Distance maxElevatorHeight = Inches.of(70);//

  public final SparkMax leftMotor = new SparkMax(CANIDConstants.leftElevatorID, MotorType.kBrushless);
  public final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private SparkClosedLoopController leftClosedLoopController = leftMotor.getClosedLoopController();

  public final SparkMax rightMotor = new SparkMax(CANIDConstants.rightElevatorID, MotorType.kBrushless);
  public final RelativeEncoder rightEncoder = rightMotor.getEncoder();


  double TRAJECTORY_VEL=.8;
  double TRAJECTORY_ACCEL=2;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      TRAJECTORY_VEL, TRAJECTORY_ACCEL));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

  private ElevatorFeedforward eff;

  public int posrng;

  // private boolean shutDownElevatorPositioning;

  public boolean atUpperLimit;

  public boolean atLowerLimit;

  


  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {

    SmartDashboard.putNumber("Elevator/posconv", positionConversionFactor);
    SmartDashboard.putNumber("Elevator/posconvinch",
        Units.metersToInches(positionConversionFactor));
    SmartDashboard.putNumber("Elevator/maxspeedmps", meterspersecondsprocket);// 4800 rpm
    // SmartDashboard.putNumber("Elevator/gear", kElevatorGearing);

    // SmartDashboard.putNumber("Elevator/velconv", velocityConversionFactor);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig

        .smartCurrentLimit(40)

        .closedLoopRampRate(0.25)

            .closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        .p(elevatorKp)

        .outputRange(-1, 1);

    leftConfig.

        encoder.

        positionConversionFactor(positionConversionFactor)

        .velocityConversionFactor(velocityConversionFactor);

    rightConfig

        .follow(CANIDConstants.leftElevatorID, true)

        .smartCurrentLimit(40)

        .closedLoopRampRate(0.25).closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        // Set PID values for position control

        .p(elevatorKp)

        .outputRange(-1, 1);

    rightConfig.encoder

        .positionConversionFactor(positionConversionFactor)

        .velocityConversionFactor(velocityConversionFactor);

    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    eff = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv, elevatorKa);

    resetPosition(minElevatorHeight.in(Meters));

    setGoalMeters(minElevatorHeight.in(Meters));
  }

  /**
   * Advance the simulation
   */
  public void simulationPeriodic() {

    SmartDashboard.putNumber("Elevator/positionleft", getLeftPositionMeters());
    SmartDashboard.putNumber("Elevator/Velleft", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator/positionright", rightEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Velright", rightEncoder.getVelocity());

    SmartDashboard.putNumber("Elevator/APPO", leftMotor.getAppliedOutput());

  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop() {
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);

  }

   public void setGoalMeters(double targetMeters) {
    m_goal.position = targetMeters-Units.inchesToMeters(elevatorToGround);
  }

  public void setGoalInches(double targetInches) {
    double targetMeters = Units.inchesToMeters(targetInches);
    m_goal.position = targetMeters - elevatorToGround;
    currentSetpoint.position=leftEncoder.getPosition();
    // inPositionCtr = 0;
  }

  public double getLeftPositionMeters() {
    if (RobotBase.isReal()) {
      return leftEncoder.getPosition();
    } else
      return leftEncoder.getPosition();// + minElevatorHeight.in(Meters);
  }

  public double getLeftPositionInches() {
    return Units.metersToInches(getLeftPositionMeters());
  }

  public void position() {

    posrng++;

    // Send setpoint to spark max controller
    nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

    double leftff = eff.calculateWithVelocities(currentSetpoint.velocity, nextSetpoint.velocity);

    currentSetpoint = nextSetpoint;

    SmartDashboard.putNumber("Elevator/ff", leftff);

    leftClosedLoopController.setReference(
        nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, leftff, ArbFFUnits.kVoltage);
    

    SmartDashboard.putNumber("ElTrp/setpos", nextSetpoint.position);

    SmartDashboard.putNumber("ElTrp/setvel", nextSetpoint.velocity);
  }

  public void resetPosition(double val) {
    leftEncoder.setPosition(val);
    rightEncoder.setPosition(val);
    SmartDashboard.putNumber("Elevator/posaftereset", getLeftPositionMeters());
  }

  @Override
  public void periodic() {

    atUpperLimit = getLeftPositionMeters() >= maxElevatorHeight.in(Meters)
        || rightEncoder.getPosition() >= maxElevatorHeight.in(Meters);

    atLowerLimit = getLeftPositionMeters() <= minElevatorHeight.in(Meters)
        || (rightEncoder.getPosition() <= minElevatorHeight.in(Meters)
            && RobotBase.isReal());

    SmartDashboard.putBoolean("Elevator/onUpperLimit", atUpperLimit);
    SmartDashboard.putBoolean("Elevator/onLowerLimit", atLowerLimit);
    SmartDashboard.putNumber("Elevator/volts", leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Elevator/posgoal", m_goal.position);
    SmartDashboard.putNumber("Elevator/poserror", m_goal.position - getLeftPositionMeters());

  }


}