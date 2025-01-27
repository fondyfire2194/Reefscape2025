// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SimulationRobotConstants;

public class ElevatorArmSim extends SubsystemBase implements AutoCloseable {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
  // Standard classes for controlling our elevator

  // Simulation classes help us simulate what'shboard going on, including gravity.
  private final ElevatorSim m_elevatorSim;
  private final SparkMaxSim m_elevatorMotorSim;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);

  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 25, 0);

  private final MechanismLigament2d m_elevatorLig2d;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private final SingleJointedArmSim m_armSim;

  private final SparkMaxSim m_armMotorSim;

  private final MechanismLigament2d m_armLig2d;

  /** Subsystem constructor. */
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;

  // public final double iixelsPerMeter = ;

  public ElevatorArmSim(ElevatorSubsystem elevator, ArmSubsystem arm) {
    m_elevator = elevator;
    m_arm = arm;
    m_armMotorSim = new SparkMaxSim(m_arm.armMotor, m_armGearbox);
    m_elevatorMotorSim = new SparkMaxSim(m_elevator.m_leftMotor, m_elevatorGearbox);
    m_elevatorSim = new ElevatorSim(
        m_elevatorGearbox,
        m_elevator.kElevatorGearing,
        SimulationRobotConstants.kCarriageMass,
        SimulationRobotConstants.kElevatorDrumRadius,
        m_elevator.minElevatorHeightMeters,
        m_elevator.maxElevatorHeightMeters,
        true,
        m_elevator.minElevatorHeightMeters,
        0.0,
        0.0);

    m_elevatorLig2d = m_mech2dRoot.append(
        new MechanismLigament2d("Elevator",
            (m_elevator.minElevatorHeightMeters + m_elevatorSim.getPositionMeters())
                * SimulationRobotConstants.kPixelsPerMeter,
            90));

    m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        m_arm.gearReduction,
        SingleJointedArmSim.estimateMOI(m_arm.armLength, m_arm.armMass),
        m_arm.armLength * SimulationRobotConstants.kPixelsPerMeter,
        m_arm.minAngle.in(Radians),
        m_arm.maxAngle.in(Radians),
        true,
        m_arm.minAngle.in(Radians),
        0.01,
        0.0 // Add noise with a std-dev of 1 tick
    );

    m_armLig2d = m_elevatorLig2d.append(
        new MechanismLigament2d(
            "Arm",
            SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
            90 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)));
    // Publish Mechanism2d to SmartDashboard

    elevator.resetPosition(0);
    SmartDashboard.putData("Elevator Sim", m_mech2d);

  }

  @Override
  public void periodic() {
    // Update mechanism2d
    m_elevatorLig2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * m_elevator.maxElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * m_elevator.m_leftMotor.getEncoder().getPosition());
    m_armLig2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.radiansToDegrees(
                    m_arm.armMotor.getEncoder().getPosition()))
            - 90 // subtract 90 degrees to account for the elevator
    );

  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    SmartDashboard.putNumber("Elevator/APPO", m_elevatorMotorSim.getAppliedOutput());
    SmartDashboard.putNumber("Elevator/sim velocity", m_elevatorSim.getVelocityMetersPerSecond());

    m_elevatorSim.setInput(m_elevatorMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    // elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    double vmps = m_elevatorSim.getVelocityMetersPerSecond();

    if (m_elevatorMotorSim.getAppliedOutput() == 0)
      vmps = 0;

    // Iterate the elevator and arm SPARK simulations
    m_elevatorMotorSim.iterate(
        vmps * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);

    vmps = m_armMotorSim.getAppliedOutput();

    if (m_armMotorSim.getAppliedOutput() == 0)
      vmps = 0;

    m_armMotorSim.iterate(
        vmps * 60,
        RobotController.getBatteryVoltage(),
        0.02);

    if (m_arm.armMotor.getEncoder().getPosition() < m_arm.minAngle.in(Radians))
      m_arm.resetEncoder(m_arm.minAngle.in(Radians));

    // SimBattery is updated in Robot.java
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_elevator.elevatorCurrentTarget = m_elevator.getLeftPositionMeters();
    m_elevator.m_leftMotor.setVoltage(0.0);
  }

  @Override
  public void close() {
    m_elevator.m_leftMotor.close();
    m_mech2d.close();
  }
}
