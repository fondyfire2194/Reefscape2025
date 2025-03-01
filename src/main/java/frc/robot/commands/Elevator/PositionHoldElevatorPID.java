// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.thethriftybot.ThriftyNova.PIDSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionHoldElevatorPID extends Command {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem m_arm;
    private PIDController pidController;
    private double kp;
    private double ki;
    private double kd;
    private double izone;
    private double minIntegral;
    private double maxIntegral;
    private double tolerance;
    private double lastGoal;

    public PositionHoldElevatorPID(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        m_arm = arm;
        pidController = new PIDController(kp, ki, kd);
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = elevator.getLeftPositionMeters();

        elevator.setGoalMeters(temp);
    }

    @Override
    public void execute() {

        double goal = elevator.getGoalMeters();

        if (goal != lastGoal) {

            pidController.setSetpoint(goal);

            lastGoal = goal;
        }

        double armPos = m_arm.armMotor.getEncoder().getPosition();

        SmartDashboard.putNumber("Elevator/armpos", armPos);

        elevator.armClear = Units.radiansToDegrees(armPos) < elevator.armClearAngle;

        double temp = pidController.calculate(armPos);

        double mps = temp * elevator.maxVelocityMPS / 4;

        mps = MathUtil.clamp(mps, -1,1);

        elevator.leftClosedLoopController.setReference(mps, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}