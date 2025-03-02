// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionHoldElevatorPID extends Command {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem m_arm;
    private PIDController pidController;
    private SlewRateLimiter slr;
    private double kp = .4;
    private double ki;
    private double kd;
    private double izone;
    private double minIntegral;
    private double maxIntegral;
    private double tolerance = Units.inchesToMeters(1);
    private double lastGoal;
    private double slrp = 20;
    private double slrn = -100000;
    private double maxrate = 2;

    public PositionHoldElevatorPID(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        m_arm = arm;
        pidController = new PIDController(kp, ki, kd);
        slr = new SlewRateLimiter(slrp, slrn, 0);
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = elevator.getLeftPositionMeters();
        elevator.pidGoalMeters = temp;
    }

    @Override
    public void execute() {

        double goal = elevator.pidGoalMeters;

        if (goal != lastGoal) {
            pidController.setSetpoint(goal);
            slr.reset(0);
            lastGoal = goal;
        }

        double armPos = m_arm.armMotor.getEncoder().getPosition();

        SmartDashboard.putNumber("Elevator/armpos", armPos);

        elevator.armClear = Units.radiansToDegrees(armPos) < elevator.armClearAngle;

        double temp = pidController.calculate(elevator.getLeftPositionMeters());

        double mps = slr.calculate(temp) * elevator.maxVelocityMPS;

        mps = MathUtil.clamp(mps, -maxrate, maxrate);

        elevator.runAtVelocity(mps);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}