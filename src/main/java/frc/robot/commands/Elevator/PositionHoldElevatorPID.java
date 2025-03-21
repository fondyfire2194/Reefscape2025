// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

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
    private double kp = 14.;
    private double ki = 0;
    private double kd = 0.2;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxrate = 2;
    private boolean toggle;

    public PositionHoldElevatorPID(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        m_arm=arm;
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
        SmartDashboard.putData(" Elevator/PID/controller", pidController);
    }

    @Override
    public void execute() {

        
        elevator.armClear = checkArmClear();

        SmartDashboard.putNumber("Elevator/poslim",
        Units.radiansToDegrees(m_arm.armMotor.getEncoder().getPosition()));

        toggle = !toggle;

        elevator.nextSetpoint = elevator.m_profile.calculate(.02, elevator.currentSetpoint, elevator.m_goal);

        double mps = pidController.calculate(elevator.getLeftPositionMeters(), elevator.nextSetpoint.position);

        if (toggle) {
            SmartDashboard.putNumber("Elevator/PID/goalpos", elevator.m_goal.position);
            SmartDashboard.putNumber("Elevator/PID/currsetpos", elevator.currentSetpoint.position);
            SmartDashboard.putNumber("Elevator/PID/currsetvel", elevator.currentSetpoint.velocity);
            SmartDashboard.putNumber("Elevator/PID/setpos", elevator.nextSetpoint.position);

        } else {
            SmartDashboard.putNumber("Elevator/PID/position", elevator.getLeftPositionMeters());
            SmartDashboard.putNumber("Elevator/PID/setvel", elevator.nextSetpoint.velocity);
            SmartDashboard.putNumber("Elevator/PID/mps", mps);
            SmartDashboard.putNumber("Elevator/PID/mpsRead", elevator.getLeftVelocityMetersPerSecond());
            SmartDashboard.putNumber("Elevator/PID/poserror", pidController.getError());
            SmartDashboard.putBoolean("Elevator/PID/poserror", pidController.atSetpoint());
        }
        mps = MathUtil.clamp(mps, -maxrate, maxrate);

        SmartDashboard.putNumber("Elevator/PID/mpsclamped", mps);

        elevator.runAtVelocity(mps);

        elevator.currentSetpoint = elevator.nextSetpoint;

        
    }



    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean checkArmClear() {
        return Units.radiansToDegrees(m_arm.armMotor.getEncoder().getPosition()) < elevator.armClearAngleDeg;
    }


}