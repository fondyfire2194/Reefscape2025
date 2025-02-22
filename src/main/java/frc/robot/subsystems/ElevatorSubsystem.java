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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.CANIDConstants;
import monologue.Annotations.Log;
import monologue.Logged;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase implements Logged {

  public final double kElevatorGearing = ((62. * 22.) / (9. * 16.));// 9.4722222

  public final double kElevatorDrumRadiusInches = 1.757;

  public final double kElevatorDrumRadiusMeters = Units.inchesToMeters(kElevatorDrumRadiusInches);

  public final int sprocketTeeth = 22;

  public final double metersPerSprocketRev = kElevatorDrumRadiusMeters * 2 * Math.PI;// 6.3 inches

  public final double meterspersecondsprocket = metersPerSprocketRev * (80. / kElevatorGearing);

  public double metersPerMotorRev = (metersPerSprocketRev / kElevatorGearing);//

  public double positionConversionFactor = metersPerMotorRev;
  public double velocityConversionFactor = positionConversionFactor / 60;

  public double elevatorToGroundInches = Units.inchesToMeters(6);

  public double maxVelocityMPS = meterspersecondsprocket;

  public final double elevatorKp = .95;
  public final double elevatorKi = 0;
  public final double elevatorKd = 0;

  public final double elevatorKs = .06;
  public final double elevatorKg = 1.1;
  public final double elevatorKv = 8;
  public final double elevatorKa = 0.08;

  public final double kCarriageMass = Units.lbsToKilograms(1); // kg

  public final Distance minElevatorHeight = Inches.of(5);
  public final Distance maxElevatorHeight = Inches.of(70);//

  public final SparkMax leftMotor = new SparkMax(CANIDConstants.leftElevatorID, MotorType.kBrushless);
  public final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private SparkClosedLoopController leftClosedLoopController = leftMotor.getClosedLoopController();

  public final SparkMax rightMotor = new SparkMax(CANIDConstants.rightElevatorID, MotorType.kBrushless);
  public final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  @Log(key = "alert warning")
  private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
  @Log(key = "alert error")
  private Alert allErrors = new Alert("AllErrors", AlertType.kError);
  @Log(key = "alert sticky fault")
  private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);
  double TRAJECTORY_VEL = .8;
  double TRAJECTORY_ACCEL = 2;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      TRAJECTORY_VEL, TRAJECTORY_ACCEL));

  @Log.NT(key = "goal")
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

  @Log.NT(key = "setpoint")
  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

  private ElevatorFeedforward eff;

  public int posrng;

  // SysId Routine and seutp
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  public final Trigger atMin = new Trigger(() -> getLeftPositionMeters() <= .1);

  public final Trigger atMax = new Trigger(() -> getLeftPositionMeters() > (maxElevatorHeight.in(Meters) - .2));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(Volts.per(Second).of(1),
          Volts.of(7),
          Seconds.of(10)),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          leftMotor::setVoltage,
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getLeftPositionMeters(),
                    Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                .linearVelocity(m_velocity.mut_replace(getLeftVelocityMetersPerSecond(),
                    MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
          },
          this));

  Alert elevatorError = new Alert("ElevetorDifferenceHigh", AlertType.kError);

  @Log.NT(key = "at upper limit")
  public boolean atUpperLimit;

  @Log.NT(key = "at lower limit")
  public boolean atLowerLimit;

  @Log.NT(key = "target meters")
  private double targetMeters;

  @Log.NT(key = "left ff")
  private double leftff;

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {

    // SmartDashboard.putNumber("Elevator/posconv", positionConversionFactor);
    // SmartDashboard.putNumber("Elevator/posconvinch",
    // Units.metersToInches(positionConversionFactor));
    // SmartDashboard.putNumber("Elevator/maxspeedmps", meterspersecondsprocket);//
    // 4800 rpm
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

        .follow(CANIDConstants.leftElevatorID, false)

        .smartCurrentLimit(40)

        .closedLoopRampRate(0.25).closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

        // Set PID values for position control

        .p(elevatorKp)

        .outputRange(-1, 1);

    rightConfig

        .encoder

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

  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop() {
    leftMotor.setVoltage(0.0);
    rightMotor.setVoltage(0.0);

  }

  @Log.NT(key = "position inches")
  public double getGoalInches() {
    return m_goal.position;
  }

  public void setGoalMeters(double targetMeters) {
    m_goal.position = targetMeters - Units.inchesToMeters(elevatorToGroundInches);
  }

  public void setGoalInches(double targetInches) {
    targetMeters = Units.inchesToMeters(targetInches);
    m_goal.position = targetMeters - elevatorToGroundInches;
    SmartDashboard.putNumber("Elevator/targetMeters", targetMeters);
    currentSetpoint.position = leftEncoder.getPosition();
    // inPositionCtr = 0;
  }

  public Command runSysIdRoutine() {
    return (m_sysIdRoutine.dynamic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin))
        .andThen(Commands.print("DONE"));
  }

  public Command setGoalInchesCommand(double targetInches) {
    return Commands.runOnce(() -> setGoalInches(targetInches));
  }

  public double getLeftPositionMeters() {
    return leftEncoder.getPosition();
  }

  public double getLeftVelocityMetersPerSecond() {
    return leftEncoder.getPosition();
  }

  @Log.NT(key = "left position inches")
  public double getLeftPositionInches() {
    return Units.metersToInches(getLeftPositionMeters());
  }

  @Log.NT(key = "left applied output")
  public double getLeftAppliedOutput() {
    return leftMotor.getAppliedOutput();
  }

  public double getRightPositionMeters() {
    return leftEncoder.getPosition();
  }

  @Log.NT(key = "right position inches")
  public double getRightPositionInches() {
    return Units.metersToInches(getRightPositionMeters());
  }

  @Log.NT(key = "left position error inches")
  public double getLeftPositionError() {
    return getGoalInches() - getLeftPositionInches();
  }

  @Log.NT(key = "left right position diffinches")
  public double getLeftRightDiffInches() {
    return getLeftPositionInches() - getRightPositionInches();
  }

  public void position() {

    posrng++;

    // Send setpoint to spark max controller
    nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

    leftff = eff.calculateWithVelocities(currentSetpoint.velocity, nextSetpoint.velocity);

    currentSetpoint = nextSetpoint;

    leftClosedLoopController.setReference(
        nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, leftff, ArbFFUnits.kVoltage);
  }

  @Log.NT(key = "reset position")
  public void resetPosition(double val) {
    leftEncoder.setPosition(val);
    rightEncoder.setPosition(val);
  }

  public boolean getActiveFault() {
    return leftMotor.hasActiveFault() || rightMotor.hasActiveFault();
  }

  public boolean getStickyFault() {
    return leftMotor.hasStickyFault() || rightMotor.hasStickyFault();
  }

  public boolean getWarnings() {
    return leftMotor.hasActiveWarning() || rightMotor.hasActiveWarning();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> leftMotor.clearFaults()),
        Commands.runOnce(() -> rightMotor.clearFaults()));
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Elevator/positionleft", Units.metersToInches(getLeftPositionMeters()));
    SmartDashboard.putNumber("Elevator/Velleft", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator/positionright", Units.metersToInches(rightEncoder.getPosition()));
    SmartDashboard.putNumber("Elevator/Velright", rightEncoder.getVelocity());

    SmartDashboard.putNumber("Elevator/APPO", leftMotor.getAppliedOutput());

    allWarnings.set(getWarnings());
    allErrors.set(getActiveFault());
    allStickyFaults.set(getStickyFault());

    atUpperLimit = getLeftPositionMeters() >= maxElevatorHeight.in(Meters)
        || rightEncoder.getPosition() >= maxElevatorHeight.in(Meters);

    atLowerLimit = getLeftPositionMeters() <= minElevatorHeight.in(Meters)
        || (rightEncoder.getPosition() <= minElevatorHeight.in(Meters)
            && RobotBase.isReal());

    SmartDashboard.putNumber("Elevator/kvcalc",
        RobotController.getBatteryVoltage() / leftMotor.getEncoder().getVelocity());

    elevatorError.set(Math.abs(getLeftRightDiffInches()) > 2);
    elevatorError.setText(String.valueOf(getLeftRightDiffInches()));
  }

}